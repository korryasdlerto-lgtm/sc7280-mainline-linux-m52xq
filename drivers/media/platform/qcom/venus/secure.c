// SPDX-License-Identifier: GPL-2.0
/*
 * M52XQ: secure (content protected, non-pixel) buffers for the Venus encoder.
 *
 * The downstream driver (msm_smem.c) always allocates the encoder's
 * HAL_BUFFER_INTERNAL_PERSIST from the secure heap (ION_FLAG_CP_NON_PIXEL), behind the
 * "venus_sec_non_pixel" context bank (SID 0x2184, IOVA 0x1000000..0x25800000, VMID 0xb).
 * Here: a child device with its own IOMMU domain on that SID, pages hyp-assigned to
 * VMID_CP_NON_PIXEL through SCM and mapped at an IOVA taken from that window.
 */
#include <linux/dma-mapping.h>
#include <linux/genalloc.h>
#include <linux/iommu.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <dt-bindings/firmware/qcom,scm.h>

#include "core.h"

#define SEC_IOVA_START	0x1000000UL
#define SEC_IOVA_SIZE	0x24800000UL

int venus_secure_init(struct venus_core *core)
{
	struct platform_device_info info;
	struct platform_device *pdev;
	struct device_node *np;
	int ret;

	np = of_get_child_by_name(core->dev->of_node, "video-secure-nonpixel");
	if (!np)
		return 0;

	memset(&info, 0, sizeof(info));
	info.fwnode = &np->fwnode;
	info.parent = core->dev;
	info.name = np->name;
	info.dma_mask = DMA_BIT_MASK(32);

	pdev = platform_device_register_full(&info);
	if (IS_ERR(pdev)) {
		of_node_put(np);
		return PTR_ERR(pdev);
	}

	pdev->dev.of_node = np;

	ret = of_dma_configure(&pdev->dev, np, true);
	if (ret) {
		dev_err(core->dev, "M52SEC: dma configure failed: %d\n", ret);
		goto err_unregister;
	}

	core->sec.dev = &pdev->dev;

	core->sec.dom = iommu_paging_domain_alloc(core->sec.dev);
	if (IS_ERR(core->sec.dom)) {
		ret = PTR_ERR(core->sec.dom);
		core->sec.dom = NULL;
		dev_err(core->dev, "M52SEC: iommu domain alloc failed: %d\n", ret);
		goto err_unregister;
	}

	ret = iommu_attach_device(core->sec.dom, core->sec.dev);
	if (ret) {
		dev_err(core->dev, "M52SEC: iommu attach failed: %d\n", ret);
		goto err_free_dom;
	}

	core->sec.pool = gen_pool_create(PAGE_SHIFT, -1);
	if (!core->sec.pool) {
		ret = -ENOMEM;
		goto err_detach;
	}

	ret = gen_pool_add(core->sec.pool, SEC_IOVA_START, SEC_IOVA_SIZE, -1);
	if (ret)
		goto err_pool;

	of_node_put(np);
	dev_info(core->dev, "M52SEC: secure non-pixel domain ready (SID via %s)\n",
		 dev_name(core->sec.dev));
	return 0;

err_pool:
	gen_pool_destroy(core->sec.pool);
	core->sec.pool = NULL;
err_detach:
	iommu_detach_device(core->sec.dom, core->sec.dev);
err_free_dom:
	iommu_domain_free(core->sec.dom);
	core->sec.dom = NULL;
err_unregister:
	core->sec.dev = NULL;
	platform_device_unregister(pdev);
	of_node_put(np);
	return ret;
}

void venus_secure_deinit(struct venus_core *core)
{
	if (!core->sec.dev)
		return;

	if (core->sec.pool) {
		gen_pool_destroy(core->sec.pool);
		core->sec.pool = NULL;
	}

	if (core->sec.dom) {
		iommu_detach_device(core->sec.dom, core->sec.dev);
		iommu_domain_free(core->sec.dom);
		core->sec.dom = NULL;
	}

	platform_device_unregister(to_platform_device(core->sec.dev));
	core->sec.dev = NULL;
}

static int sec_assign(struct page *page, size_t size, bool to_secure)
{
	struct qcom_scm_vmperm perm;
	u64 src;

	if (to_secure) {
		src = BIT(QCOM_SCM_VMID_HLOS);
		perm.vmid = QCOM_SCM_VMID_CP_NON_PIXEL;
		perm.perm = QCOM_SCM_PERM_RW;
	} else {
		src = BIT(QCOM_SCM_VMID_CP_NON_PIXEL);
		perm.vmid = QCOM_SCM_VMID_HLOS;
		perm.perm = QCOM_SCM_PERM_RWX;
	}

	return qcom_scm_assign_mem(page_to_phys(page), size, &src, &perm, 1);
}

/*
 * The memory is zeroed and its cache lines are cleaned before the assignment and it must not be touched by the CPU
 * afterwards. It is built from single pages (a contiguous order-6 allocation failed under memory pressure); every page
 * is assigned and mapped on its own, the IOVA range is contiguous.
 */
int venus_secure_alloc(struct venus_core *core, size_t size, struct venus_sec_buf *b)
{
	unsigned long iova;
	unsigned int i, mapped = 0, assigned = 0;
	int ret = -ENOMEM;

	if (!core->sec.dev || !core->sec.dom || !core->sec.pool)
		return -ENODEV;

	memset(b, 0, sizeof(*b));
	b->size = ALIGN(size, PAGE_SIZE);
	b->npages = b->size >> PAGE_SHIFT;
	b->pages = kcalloc(b->npages, sizeof(*b->pages), GFP_KERNEL);
	if (!b->pages)
		return -ENOMEM;

	for (i = 0; i < b->npages; i++) {
		dma_addr_t d;

		b->pages[i] = alloc_page(GFP_KERNEL | __GFP_ZERO);
		if (!b->pages[i])
			goto err_pages;

		/* clean the dirty lines of the zeroing: a late write-back into a protected page is a violation */
		d = dma_map_page(core->dev, b->pages[i], 0, PAGE_SIZE, DMA_BIDIRECTIONAL);
		if (dma_mapping_error(core->dev, d)) {
			ret = -EIO;
			goto err_pages;
		}
		dma_unmap_page(core->dev, d, PAGE_SIZE, DMA_BIDIRECTIONAL);
	}

	iova = gen_pool_alloc(core->sec.pool, b->size);
	if (!iova)
		goto err_pages;

	for (i = 0; i < b->npages; i++) {
		ret = sec_assign(b->pages[i], PAGE_SIZE, true);
		if (ret) {
			dev_err(core->dev, "M52SEC: assign to CP_NON_PIXEL failed: %d (page %u/%u)\n", ret, i, b->npages);
			goto err_unwind;
		}
		assigned++;
	}

	for (i = 0; i < b->npages; i++) {
		ret = iommu_map(core->sec.dom, iova + ((unsigned long)i << PAGE_SHIFT), page_to_phys(b->pages[i]),
				PAGE_SIZE, IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
		if (ret) {
			dev_err(core->dev, "M52SEC: iommu_map failed: %d\n", ret);
			goto err_unwind;
		}
		mapped++;
	}

	b->da = iova;
	dev_info(core->dev, "M52SEC: secure buffer da=0x%lx size=0x%zx (%u pages)\n", iova, b->size, b->npages);
	return 0;

err_unwind:
	if (mapped)
		iommu_unmap(core->sec.dom, iova, (size_t)mapped << PAGE_SHIFT);
	for (i = 0; i < assigned; i++) {
		if (sec_assign(b->pages[i], PAGE_SIZE, false)) {
			/* the page stays protected: leak it rather than hand it back to the CPU */
			b->pages[i] = NULL;
		}
	}
	gen_pool_free(core->sec.pool, iova, b->size);
err_pages:
	for (i = 0; i < b->npages; i++) {
		if (b->pages[i])
			__free_page(b->pages[i]);
	}
	kfree(b->pages);
	memset(b, 0, sizeof(*b));
	return ret;
}

void venus_secure_free(struct venus_core *core, struct venus_sec_buf *b)
{
	unsigned int i;

	if (!b->pages)
		return;

	iommu_unmap(core->sec.dom, b->da, b->size);
	gen_pool_free(core->sec.pool, b->da, b->size);

	for (i = 0; i < b->npages; i++) {
		if (sec_assign(b->pages[i], PAGE_SIZE, false)) {
			dev_err(core->dev, "M52SEC: reclaim failed, leaking a page\n");
			continue;
		}
		__free_page(b->pages[i]);
	}

	kfree(b->pages);
	memset(b, 0, sizeof(*b));
}
