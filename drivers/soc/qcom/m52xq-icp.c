// SPDX-License-Identifier: GPL-2.0-only
/*
 * M52XQ (SM7325): bring-up of the camera ICP (A5 core, first-generation ICP) for the BPS/IPE offline engines.
 *
 * Mirrors the downstream sequence (techpack/camera/drivers/cam_icp: a5_core.c, hfi.c, cam_icp_hw_mgr.c):
 * the HLOS copies CAMERA_ICP_170.elf into the "firmware" carveout (no TrustZone/PAS on this generation),
 * maps it at ICP IOVA 0, starts the A5 through its CSR block, sets up the HFI queue table and waits for
 * HOST_INIT_RESPONSE, then sends SYS_INIT and PING.
 *
 * Only started on request: echo 1 > /sys/kernel/debug/m52xq-icp/boot ; cat /sys/kernel/debug/m52xq-icp/status
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/elf.h>
#include <linux/firmware.h>
#include <linux/interconnect.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

/* A5 CSR block ("a5_sierra" reg, downstream A5_SIERRA_BASE) */
#define A5_NSEC_RESET		0x04
#define  A5_FUNC_RESET		BIT(4)
#define A5_CONTROL		0x08
#define  A5_CPU_EN		BIT(9)
#define  A5_WAKE_UP_EN		BIT(4)
#define  A5_EN_CLKGATE_WFI	BIT(12)
#define  A5_FULL_CPU_EN		(A5_CPU_EN | A5_WAKE_UP_EN | A5_EN_CLKGATE_WFI)
#define A5_A2HOSTINTEN		0x10
#define  A5_WDT_WS0EN		BIT(1)
#define  A5_A2HOSTINT_EN	BIT(0)
#define A5_HOST_INT_CLR		0x18
#define A5_HOST_INT_STATUS	0x1c
#define A5_HOST2ICPINT		0x30
#define A5_CSR_ACCESS		0x3c
#define A5_STATUS		0x200
#define  A5_STANDBYWFI		BIT(7)

/* HFI general purpose registers: A5 CSR + 0x40 + n * 4 */
#define HFI_REG(n)		(0x40 + (n) * 4)
#define HFI_FW_VERSION		HFI_REG(1)
#define HFI_INIT_REQUEST	HFI_REG(2)
#define HFI_INIT_RESPONSE	HFI_REG(3)
#define HFI_SHMEM_PTR		HFI_REG(4)
#define HFI_SHMEM_SIZE		HFI_REG(5)
#define HFI_QTBL_PTR		HFI_REG(6)
#define HFI_UNCACHED_HEAP_PTR	HFI_REG(7)
#define HFI_UNCACHED_HEAP_SIZE	HFI_REG(8)
#define HFI_SFR_PTR		HFI_REG(10)
#define HFI_QDSS_IOVA		HFI_REG(11)
#define HFI_QDSS_SIZE		HFI_REG(12)
#define HFI_IO_IOVA		HFI_REG(13)
#define HFI_IO_SIZE		HFI_REG(14)
#define HFI_IO2_IOVA		HFI_REG(15)
#define HFI_IO2_SIZE		HFI_REG(16)

#define HFI_CMD_SYS_INIT	0x10001
#define HFI_CMD_SYS_PC_PREP	0x10002
#define HFI_CMD_SYS_PING	0x10005
#define HFI_MSG_SYS_PC_PREP_DONE	0x20002
#define HFI_MSG_SYS_INIT_DONE	0x20001
#define HFI_MSG_SYS_PING_ACK	0x20006
/* IPE/BPS domain (HFI_DOMAIN_BASE_IPE_BPS = 1 << 24) */
#define HFI_CMD_IPEBPS_CREATE_HANDLE		0x1010008
#define HFI_CMD_IPEBPS_ASYNC_COMMAND_DIRECT	0x101000a
#define HFI_CMD_IPEBPS_ASYNC_COMMAND_INDIRECT	0x101000e
#define HFI_MSG_IPEBPS_ASYNC_COMMAND_INDIRECT_ACK	0x102000e
#define HFI_IPEBPS_CMD_OPCODE_BPS_CONFIG_IO	0x1
#define HFI_IPEBPS_CMD_OPCODE_IPE_CONFIG_IO	0x5
#define HFI_IPEBPS_CMD_OPCODE_MEM_MAP		0xe
#define HFI_IPEBPS_CMD_OPCODE_MEM_UNMAP		0xf
#define HFI_MSG_IPEBPS_CREATE_HANDLE_ACK	0x1020008
#define HFI_MSG_IPEBPS_ASYNC_COMMAND_DIRECT_ACK	0x102000a
#define HFI_IPEBPS_CMD_OPCODE_BPS_DESTROY	0x4
#define HFI_IPEBPS_CMD_OPCODE_IPE_DESTROY	0x8

/* stock cam-smmu "icp" IOVA map (yupik DT) */
#define ICP_FW_IOVA		0x00000000
#define ICP_SHARED_IOVA		0x07400000
#define ICP_SHARED_LEN		0x09600000
#define ICP_SECHEAP_IOVA	0x10a00000
#define ICP_SECHEAP_LEN		0x00100000
#define ICP_QDSS_IOVA		0x10b00000
#define ICP_QDSS_LEN		0x00100000
#define ICP_IO_IOVA		0x10c00000
#define ICP_IO_DISCARD		0xdff00000
#define ICP_IO_DISCARD_LEN	0x00300000
#define ICP_IO_END		(0x10c00000ULL + 0xee300000ULL)

/* host memory in the shared region: one 1 MiB block */
#define ICP_HOSTMEM_LEN		SZ_1M
/* buffer for CONFIG_IO / FRAME_PROCESS payloads, filled from userspace through debugfs cfgbuf */
#define ICP_IOBUF_IOVA		ICP_IO_IOVA
#define ICP_IOBUF_LEN		SZ_64K
#define OFF_QTBL		0x00000
#define OFF_CMDQ		0x01000
#define OFF_MSGQ		0x02000
#define OFF_DBGQ		0x03000
#define OFF_SFR			0x1d000
#define CMDQ_BYTES		4096
#define MSGQ_BYTES		4096
#define DBGQ_BYTES		102400
#define SFR_BYTES		4096

struct hfi_qtbl_hdr {
	u32 version, size, qhdr0_offset, qhdr_size, num_q, num_active_q;
} __packed;

/* downstream struct hfi_q_hdr: every field is followed by 15 padding words (64-byte stride) */
struct hfi_q_hdr {
	u32 dummy[15];
	u32 status;
	u32 dummy1[15];
	u32 start_addr;
	u32 dummy2[15];
	u32 type;
	u32 dummy3[15];
	u32 q_size;
	u32 dummy4[15];
	u32 pkt_size;
	u32 dummy5[15];
	u32 pkt_drop_cnt;
	u32 dummy6[15];
	u32 rx_wm;
	u32 dummy7[15];
	u32 tx_wm;
	u32 dummy8[15];
	u32 rx_req;
	u32 dummy9[15];
	u32 tx_req;
	u32 dummy10[15];
	u32 rx_irq_status;
	u32 dummy11[15];
	u32 tx_irq_status;
	u32 dummy12[15];
	u32 read_idx;
	u32 dummy13[15];
	u32 write_idx;
	u32 dummy14[15];
};

struct hfi_qtbl {
	struct hfi_qtbl_hdr hdr;
	struct hfi_q_hdr q[3];
};

enum { Q_CMD, Q_MSG, Q_DBG };

struct m52xq_icp {
	struct device *dev;
	void __iomem *csr;		/* "sierra" */
	struct clk_bulk_data *clks;
	int num_clks;
	struct icc_path *icc, *icc_sf;
	struct dev_pm_domain_list *pd_list;
	int irq;
	struct iommu_domain *domain;
	phys_addr_t fw_phys;
	size_t fw_size;
	void *fw_kva;
	struct device *memdev;		/* child without IOMMU: dma_alloc_coherent gives the physical address */
	void *host_kva;
	dma_addr_t host_phys;
	void *secheap_kva, *qdss_kva;
	dma_addr_t secheap_phys, qdss_phys;
	void *iobuf_kva;
	dma_addr_t iobuf_phys;
	size_t iobuf_used;
	bool powered, mapped, running;
	u32 fw_version, api_ver, dev_ver, last_irq;
	wait_queue_head_t wq;
	atomic_t irqs;
	char log[2048];
	size_t log_len;
	struct mutex lock;
	struct dentry *dbg;
	struct miscdevice misc;
	int users;
};

static char *fw_name = "qcom/sm7325/samsung/m52xq/CAMERA_ICP_170.elf";
module_param(fw_name, charp, 0644);
static unsigned int qos_val = 0xa0a;
module_param(qos_val, uint, 0644);
static unsigned long icp_rate = 400000000;
module_param(icp_rate, ulong, 0644);
/* downstream yupik clock-rates at svs: bps 400 MHz, ipe 430 MHz */
static unsigned long bps_rate = 400000000;
module_param(bps_rate, ulong, 0644);
static unsigned long ipe_rate = 430000000;
module_param(ipe_rate, ulong, 0644);

static void icp_log(struct m52xq_icp *icp, const char *fmt, ...)
{
	va_list ap;
	int n;

	if (icp->log_len > sizeof(icp->log) - 256)
		icp->log_len = 0;
	va_start(ap, fmt);
	n = vscnprintf(icp->log + icp->log_len, sizeof(icp->log) - icp->log_len, fmt, ap);
	va_end(ap);
	icp->log_len += n;
	dev_info(icp->dev, "%s", icp->log + icp->log_len - n);
}

static irqreturn_t icp_irq(int irq, void *data)
{
	struct m52xq_icp *icp = data;
	u32 st;

	if (!icp->powered)
		return IRQ_NONE;
	st = readl(icp->csr + A5_HOST_INT_STATUS);
	writel(st, icp->csr + A5_HOST_INT_CLR);
	icp->last_irq = st;
	atomic_inc(&icp->irqs);
	wake_up(&icp->wq);
	return IRQ_HANDLED;
}

static int icp_power(struct m52xq_icp *icp, bool on)
{
	int ret;

	if (on == icp->powered)
		return 0;
	if (on) {
		ret = pm_runtime_resume_and_get(icp->dev);
		if (ret)
			return ret;
		ret = icc_set_bw(icp->icc, 100000, 400000);
		if (ret)
			dev_warn(icp->dev, "icc_set_bw: %d\n", ret);
		if (icp->icc_sf)
			icc_set_bw(icp->icc_sf, 400000, 1000000);
		ret = clk_bulk_prepare_enable(icp->num_clks, icp->clks);
		if (ret) {
			pm_runtime_put_sync(icp->dev);
			return ret;
		}
		icp->powered = true;
	} else {
		icp->powered = false;
		clk_bulk_disable_unprepare(icp->num_clks, icp->clks);
		icc_set_bw(icp->icc, 0, 0);
		if (icp->icc_sf)
			icc_set_bw(icp->icc_sf, 0, 0);
		pm_runtime_put_sync(icp->dev);
	}
	return 0;
}

static int icp_map(struct m52xq_icp *icp)
{
	int ret;

	if (icp->mapped)
		return 0;
	ret = iommu_map(icp->domain, ICP_FW_IOVA, icp->fw_phys, icp->fw_size,
			IOMMU_READ | IOMMU_WRITE | IOMMU_PRIV, GFP_KERNEL);
	if (ret)
		return ret;
	ret = iommu_map(icp->domain, ICP_SHARED_IOVA, icp->host_phys, ICP_HOSTMEM_LEN,
			IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
	if (ret)
		goto unmap_fw;
	ret = iommu_map(icp->domain, ICP_SECHEAP_IOVA, icp->secheap_phys, ICP_SECHEAP_LEN,
			IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
	if (ret)
		goto unmap_host;
	ret = iommu_map(icp->domain, ICP_QDSS_IOVA, icp->qdss_phys, ICP_QDSS_LEN,
			IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
	if (ret)
		goto unmap_heap;
	ret = iommu_map(icp->domain, ICP_IOBUF_IOVA, icp->iobuf_phys, ICP_IOBUF_LEN,
			IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
	if (ret) {
		iommu_unmap(icp->domain, ICP_QDSS_IOVA, ICP_QDSS_LEN);
		goto unmap_heap;
	}
	icp->mapped = true;
	return 0;
unmap_heap:
	iommu_unmap(icp->domain, ICP_SECHEAP_IOVA, ICP_SECHEAP_LEN);
unmap_host:
	iommu_unmap(icp->domain, ICP_SHARED_IOVA, ICP_HOSTMEM_LEN);
unmap_fw:
	iommu_unmap(icp->domain, ICP_FW_IOVA, icp->fw_size);
	return ret;
}

static int icp_load_fw(struct m52xq_icp *icp)
{
	const struct firmware *fw;
	const struct elf32_hdr *eh;
	const struct elf32_phdr *ph;
	u32 i, end, loaded = 0;
	int ret;

	ret = request_firmware(&fw, fw_name, icp->dev);
	if (ret) {
		icp_log(icp, "no firmware %s: %d\n", fw_name, ret);
		return ret;
	}
	eh = (const void *)fw->data;
	if (fw->size < sizeof(*eh) || memcmp(eh->e_ident, ELFMAG, SELFMAG) ||
	    eh->e_ident[EI_CLASS] != ELFCLASS32 || eh->e_machine != EM_ARM ||
	    eh->e_phoff + (u64)eh->e_phnum * sizeof(*ph) > fw->size) {
		icp_log(icp, "bad ICP ELF\n");
		ret = -EINVAL;
		goto out;
	}
	memset(icp->fw_kva, 0, icp->fw_size);
	ph = (const void *)(fw->data + eh->e_phoff);
	for (i = 0; i < eh->e_phnum; i++, ph++) {
		if (!ph->p_flags)
			continue;
		end = ph->p_vaddr + ph->p_memsz;
		if (end > icp->fw_size || ph->p_offset + (u64)ph->p_filesz > fw->size) {
			icp_log(icp, "segment %u vaddr 0x%x memsz 0x%x does not fit\n", i, ph->p_vaddr,
				ph->p_memsz);
			ret = -E2BIG;
			goto out;
		}
		if (ph->p_filesz)
			memcpy(icp->fw_kva + ph->p_vaddr, fw->data + ph->p_offset, ph->p_filesz);
		loaded += ph->p_filesz;
	}
	wmb();
	icp_log(icp, "firmware %s: %zu bytes, %u loaded, entry 0x%x\n", fw_name, fw->size, loaded,
		eh->e_entry);
out:
	release_firmware(fw);
	return ret;
}

static void icp_setup_qtbl(struct m52xq_icp *icp)
{
	struct hfi_qtbl *qt = icp->host_kva + OFF_QTBL;
	struct hfi_q_hdr *q;
	u32 *sfr = icp->host_kva + OFF_SFR;

	memset(icp->host_kva, 0, ICP_HOSTMEM_LEN);
	qt->hdr.version = 0xffffffff;
	qt->hdr.size = sizeof(*qt);
	qt->hdr.qhdr0_offset = sizeof(struct hfi_qtbl_hdr);
	qt->hdr.qhdr_size = sizeof(struct hfi_q_hdr);
	qt->hdr.num_q = 3;
	qt->hdr.num_active_q = 3;

	/* INTR_MODE, as downstream cam_icp_mgr_hfi_init() */
	q = &qt->q[Q_CMD];
	q->status = 1;
	q->start_addr = ICP_SHARED_IOVA + OFF_CMDQ;
	q->q_size = CMDQ_BYTES >> 2;
	q->type = Q_CMD;
	q->rx_wm = 1;
	q->tx_wm = 1;
	q->rx_req = 1;

	q = &qt->q[Q_MSG];
	q->status = 1;
	q->start_addr = ICP_SHARED_IOVA + OFF_MSGQ;
	q->q_size = MSGQ_BYTES >> 2;
	q->type = Q_MSG;
	q->rx_wm = 1;
	q->tx_wm = 1;
	q->rx_req = 1;

	q = &qt->q[Q_DBG];
	q->status = 1;
	q->start_addr = ICP_SHARED_IOVA + OFF_DBGQ;
	q->q_size = DBGQ_BYTES >> 2;
	q->type = Q_DBG;
	q->rx_wm = 1;
	q->tx_wm = 1024;

	sfr[0] = SFR_BYTES;
	wmb();
}

static int icp_write_cmd(struct m52xq_icp *icp, const u32 *pkt)
{
	struct hfi_qtbl *qt = icp->host_kva + OFF_QTBL;
	struct hfi_q_hdr *q = &qt->q[Q_CMD];
	u32 *base = icp->host_kva + OFF_CMDQ;
	u32 words = pkt[0] >> 2, wi = q->write_idx, ri = q->read_idx, space, i;

	space = wi >= ri ? q->q_size - (wi - ri) : ri - wi;
	if (!words || space <= words)
		return -ENOSPC;
	for (i = 0; i < words; i++)
		base[(wi + i) % q->q_size] = pkt[i];
	wmb();
	q->write_idx = (wi + words) % q->q_size;
	wmb();
	writel(BIT(0), icp->csr + A5_HOST2ICPINT);
	return 0;
}

/* read one packet from queue qid; returns words read (0 = empty) */
static int icp_read_q(struct m52xq_icp *icp, int qid, u32 *buf, u32 max_words)
{
	struct hfi_qtbl *qt = icp->host_kva + OFF_QTBL;
	struct hfi_q_hdr *q = &qt->q[qid];
	u32 *base = icp->host_kva + (qid == Q_DBG ? OFF_DBGQ : OFF_MSGQ);
	u32 ri = q->read_idx, wi = q->write_idx, words, i;

	rmb();
	if (ri == wi)
		return 0;
	words = base[ri % q->q_size] >> 2;
	if (!words || words > max_words) {
		q->read_idx = wi;
		return -EIO;
	}
	for (i = 0; i < words; i++)
		buf[i] = base[(ri + i) % q->q_size];
	q->read_idx = (ri + words) % q->q_size;
	wmb();
	return words;
}

static int icp_read_msg(struct m52xq_icp *icp, u32 *buf, u32 max_words)
{
	return icp_read_q(icp, Q_MSG, buf, max_words);
}

static void icp_dump_dbgq(struct m52xq_icp *icp)
{
	struct hfi_qtbl *qt = icp->host_kva + OFF_QTBL;
	struct hfi_q_hdr *q = &qt->q[Q_DBG];
	char *sfr = icp->host_kva + OFF_SFR + 4;

	icp_log(icp, "queues: cmd r/w %u/%u msg r/w %u/%u dbg r/w %u/%u, sfr '%.120s'\n",
		qt->q[Q_CMD].read_idx, qt->q[Q_CMD].write_idx, qt->q[Q_MSG].read_idx,
		qt->q[Q_MSG].write_idx, q->read_idx, q->write_idx, sfr);
}

static int icp_wait_msg(struct m52xq_icp *icp, u32 type, u32 *buf, u32 max, int ms)
{
	unsigned long end = jiffies + msecs_to_jiffies(ms);
	int n;

	do {
		n = icp_read_msg(icp, buf, max);
		if (n > 1) {
			icp_log(icp, "msg type 0x%x, %d words (irqs %d, last 0x%x)\n", buf[1], n,
				atomic_read(&icp->irqs), icp->last_irq);
			if (buf[1] == type)
				return n;
			continue;
		}
		{
			int seen = atomic_read(&icp->irqs);

			wait_event_timeout(icp->wq, atomic_read(&icp->irqs) != seen, msecs_to_jiffies(10));
		}
	} while (time_before(jiffies, end));
	return -ETIMEDOUT;
}


static void icp_stop(struct m52xq_icp *icp)
{
	u32 v, st;

	if (icp->powered && icp->running) {
		u32 pkt[2] = { 8, HFI_CMD_SYS_PC_PREP }, buf[16];
		int r = icp_write_cmd(icp, pkt);

		if (!r)
			r = icp_wait_msg(icp, HFI_MSG_SYS_PC_PREP_DONE, buf, ARRAY_SIZE(buf), 2000);
		r = readl_poll_timeout(icp->csr + A5_STATUS, st, st & A5_STANDBYWFI, 100, 10000);
		icp_log(icp, "power collapse: PC_PREP %s, A5 %s (STATUS 0x%x)\n", r ? "?" : "done",
			st & A5_STANDBYWFI ? "in WFI" : "NOT in WFI", st);
	}
	if (icp->powered) {
		writel(0, icp->csr + A5_A2HOSTINTEN);
		v = readl(icp->csr + A5_CONTROL);
		writel(v & ~(A5_CPU_EN | A5_WAKE_UP_EN), icp->csr + A5_CONTROL);
	}
	icp->running = false;
}

static int icp_boot(struct m52xq_icp *icp)
{
	u32 st = 0, buf[64];
	int ret, n;

	icp->log_len = 0;
	icp->log[0] = 0;
	atomic_set(&icp->irqs, 0);

	ret = icp_power(icp, true);
	if (ret) {
		icp_log(icp, "power on failed: %d\n", ret);
		return ret;
	}
	icp_log(icp, "powered, A5 CONTROL 0x%x STATUS 0x%x FW_VERSION reg 0x%x\n",
		readl(icp->csr + A5_CONTROL), readl(icp->csr + A5_STATUS),
		readl(icp->csr + HFI_FW_VERSION));
	writel(qos_val, icp->csr + A5_CSR_ACCESS);

	ret = icp_map(icp);
	if (ret) {
		icp_log(icp, "iommu map failed: %d\n", ret);
		goto fail;
	}
	ret = icp_load_fw(icp);
	if (ret)
		goto fail;

	/* cam_a5_power_resume() */
	writel(A5_CPU_EN, icp->csr + A5_CONTROL);
	writel(A5_FUNC_RESET, icp->csr + A5_NSEC_RESET);
	writel(A5_FULL_CPU_EN, icp->csr + A5_CONTROL);
	writel(qos_val, icp->csr + A5_CSR_ACCESS);
	icp->running = true;

	/* cam_hfi_init() */
	icp_setup_qtbl(icp);
	writel(ICP_SHARED_IOVA + OFF_QTBL, icp->csr + HFI_QTBL_PTR);
	writel(ICP_SHARED_IOVA + OFF_SFR, icp->csr + HFI_SFR_PTR);
	writel(ICP_SHARED_IOVA, icp->csr + HFI_SHMEM_PTR);
	writel(ICP_SHARED_LEN, icp->csr + HFI_SHMEM_SIZE);
	writel(ICP_SECHEAP_IOVA, icp->csr + HFI_UNCACHED_HEAP_PTR);
	writel(ICP_SECHEAP_LEN, icp->csr + HFI_UNCACHED_HEAP_SIZE);
	writel(1, icp->csr + HFI_INIT_REQUEST);
	writel(ICP_QDSS_IOVA, icp->csr + HFI_QDSS_IOVA);
	writel(ICP_QDSS_LEN, icp->csr + HFI_QDSS_SIZE);
	writel(ICP_IO_IOVA, icp->csr + HFI_IO_IOVA);
	writel(ICP_IO_DISCARD - ICP_IO_IOVA, icp->csr + HFI_IO_SIZE);
	writel(ICP_IO_DISCARD + ICP_IO_DISCARD_LEN, icp->csr + HFI_IO2_IOVA);
	writel(ICP_IO_END - (ICP_IO_DISCARD + ICP_IO_DISCARD_LEN), icp->csr + HFI_IO2_SIZE);
	wmb();

	ret = readl_poll_timeout(icp->csr + HFI_INIT_RESPONSE, st, st == 1 || st == 2, 100, 1000000);
	icp->fw_version = readl(icp->csr + HFI_FW_VERSION);
	icp_log(icp, "HOST_INIT_RESPONSE %u (%s), FW_VERSION 0x%x, A5 STATUS 0x%x, irqs %d\n", st,
		st == 1 ? "success" : ret ? "timeout" : "failed", icp->fw_version,
		readl(icp->csr + A5_STATUS), atomic_read(&icp->irqs));
	if (st != 1) {
		ret = ret ? ret : -EIO;
		icp_dump_dbgq(icp);
		goto fail;
	}

	writel(A5_WDT_WS0EN | A5_A2HOSTINT_EN, icp->csr + A5_A2HOSTINTEN);

	buf[0] = 8;
	buf[1] = HFI_CMD_SYS_INIT;
	ret = icp_write_cmd(icp, buf);
	if (!ret)
		ret = icp_wait_msg(icp, HFI_MSG_SYS_INIT_DONE, buf, ARRAY_SIZE(buf), 5000);
	if (ret < 0) {
		icp_log(icp, "SYS_INIT: no INIT_DONE (%d)\n", ret);
		icp_dump_dbgq(icp);
		goto fail;
	}
	n = ret;
	/* hfi_msg_init_done: size, pkt_type, err_type, then data (api_ver, dev_ver, num_icp_hw, ...) */
	if (n >= 5) {
		icp->api_ver = buf[3];
		icp->dev_ver = buf[4];
	}
	icp_log(icp, "INIT_DONE: err 0x%x api_ver 0x%x dev_ver 0x%x\n", n > 2 ? buf[2] : 0,
		icp->api_ver, icp->dev_ver);

	buf[0] = 16;
	buf[1] = HFI_CMD_SYS_PING;
	buf[2] = 0x4d353258;	/* "X25M" */
	buf[3] = 0;
	ret = icp_write_cmd(icp, buf);
	if (!ret)
		ret = icp_wait_msg(icp, HFI_MSG_SYS_PING_ACK, buf, ARRAY_SIZE(buf), 2000);
	icp_log(icp, "PING: %s (user data 0x%x)\n", ret > 0 ? "ACK" : "no answer", ret > 2 ? buf[2] : 0);
	icp_dump_dbgq(icp);
	return ret > 0 ? 0 : ret;

fail:
	icp_stop(icp);
	icp_power(icp, false);
	return ret;
}

static int icp_ping(struct m52xq_icp *icp)
{
	u32 buf[64];
	int ret;

	if (!icp->running)
		return -ENODEV;
	buf[0] = 16;
	buf[1] = HFI_CMD_SYS_PING;
	buf[2] = 0x4d353258 ^ (u32)jiffies;
	buf[3] = 0;
	ret = icp_write_cmd(icp, buf);
	if (!ret)
		ret = icp_wait_msg(icp, HFI_MSG_SYS_PING_ACK, buf, ARRAY_SIZE(buf), 2000);
	icp_log(icp, "PING: %s, A5 STATUS 0x%x, irqs %d\n", ret > 0 ? "ACK" : "no answer",
		readl(icp->csr + A5_STATUS), atomic_read(&icp->irqs));
	icp_dump_dbgq(icp);
	return ret > 0 ? 0 : (ret ? ret : -EIO);
}

struct hfi_cmd_create_handle {
	u32 size, pkt_type, handle_type;
	u64 user_data1, user_data2;
} __packed;

struct hfi_cmd_ipebps_destroy {
	u32 size, pkt_type, opcode;
	u64 user_data1, user_data2;
	u32 num_fw_handles;
	u32 fw_handles[1];
	u64 payload;		/* struct hfi_cmd_abort_destroy */
} __packed;

/* type: 1 = BPS, 2 = IPE real-time, 3 = IPE non-real-time; create a firmware session handle, then destroy it */
static int icp_session_test(struct m52xq_icp *icp, u32 type)
{
	struct hfi_cmd_create_handle ch = {};
	struct hfi_cmd_ipebps_destroy dh = {};
	u32 buf[64], fw_handle;
	int ret;

	if (!icp->running)
		return -ENODEV;
	ch.size = sizeof(ch);
	ch.pkt_type = HFI_CMD_IPEBPS_CREATE_HANDLE;
	ch.handle_type = type;
	ch.user_data1 = 0x5345535300000000ULL | type;	/* "SESS" */
	ret = icp_write_cmd(icp, (u32 *)&ch);
	if (!ret)
		ret = icp_wait_msg(icp, HFI_MSG_IPEBPS_CREATE_HANDLE_ACK, buf, ARRAY_SIZE(buf), 2000);
	if (ret < 4) {
		icp_log(icp, "CREATE_HANDLE type %u: no ack (%d)\n", type, ret);
		icp_dump_dbgq(icp);
		return ret < 0 ? ret : -EIO;
	}
	/* hfi_msg_create_handle_ack: size, pkt_type, err_type, fw_handle, user_data1, user_data2 */
	fw_handle = buf[3];
	icp_log(icp, "CREATE_HANDLE type %u: err 0x%x fw_handle 0x%x user 0x%08x%08x\n", type, buf[2],
		fw_handle, ret > 5 ? buf[5] : 0, ret > 4 ? buf[4] : 0);
	if (buf[2])
		return -EIO;

	dh.size = sizeof(dh);
	dh.pkt_type = HFI_CMD_IPEBPS_ASYNC_COMMAND_DIRECT;
	dh.opcode = type == 1 ? HFI_IPEBPS_CMD_OPCODE_BPS_DESTROY : HFI_IPEBPS_CMD_OPCODE_IPE_DESTROY;
	dh.user_data1 = ch.user_data1;
	dh.num_fw_handles = 1;
	dh.fw_handles[0] = fw_handle;
	ret = icp_write_cmd(icp, (u32 *)&dh);
	if (!ret)
		ret = icp_wait_msg(icp, HFI_MSG_IPEBPS_ASYNC_COMMAND_DIRECT_ACK, buf, ARRAY_SIZE(buf), 2000);
	/* hfi_msg_ipebps_async_ack: size, pkt_type, opcode, user_data1 (2), user_data2 (2), err_type */
	icp_log(icp, "DESTROY handle 0x%x: %s opcode 0x%x err 0x%x\n", fw_handle, ret > 0 ? "ack" : "no ack",
		ret > 2 ? buf[2] : 0, ret > 7 ? buf[7] : 0);
	icp_dump_dbgq(icp);
	return ret > 0 ? 0 : (ret ? ret : -EIO);
}

struct hfi_cmd_ipebps_indirect {
	u32 size, pkt_type, opcode;
	u64 user_data1, user_data2;
	u32 num_fw_handles;
	u32 fw_handles[1];
	u32 payload_iova;
} __packed;

static int icp_create_handle(struct m52xq_icp *icp, u32 type, u32 *fw_handle)
{
	struct hfi_cmd_create_handle ch = {};
	u32 buf[64];
	int ret;

	ch.size = sizeof(ch);
	ch.pkt_type = HFI_CMD_IPEBPS_CREATE_HANDLE;
	ch.handle_type = type;
	ch.user_data1 = 0x5345535300000000ULL | type;
	ret = icp_write_cmd(icp, (u32 *)&ch);
	if (!ret)
		ret = icp_wait_msg(icp, HFI_MSG_IPEBPS_CREATE_HANDLE_ACK, buf, ARRAY_SIZE(buf), 2000);
	if (ret < 4)
		return ret < 0 ? ret : -EIO;
	*fw_handle = buf[3];
	icp_log(icp, "CREATE_HANDLE type %u: err 0x%x fw_handle 0x%x\n", type, buf[2], buf[3]);
	return buf[2] ? -EIO : 0;
}

static void icp_destroy_handle(struct m52xq_icp *icp, u32 type, u32 fw_handle)
{
	struct hfi_cmd_ipebps_destroy dh = {};
	u32 buf[64];
	int ret;

	dh.size = sizeof(dh);
	dh.pkt_type = HFI_CMD_IPEBPS_ASYNC_COMMAND_DIRECT;
	dh.opcode = type == 1 ? HFI_IPEBPS_CMD_OPCODE_BPS_DESTROY : HFI_IPEBPS_CMD_OPCODE_IPE_DESTROY;
	dh.num_fw_handles = 1;
	dh.fw_handles[0] = fw_handle;
	ret = icp_write_cmd(icp, (u32 *)&dh);
	if (!ret)
		ret = icp_wait_msg(icp, HFI_MSG_IPEBPS_ASYNC_COMMAND_DIRECT_ACK, buf, ARRAY_SIZE(buf), 2000);
	icp_log(icp, "DESTROY 0x%x: %s err 0x%x\n", fw_handle, ret > 0 ? "ack" : "no ack", ret > 7 ? buf[7] : 0);
}

/* downstream hfi_cmd_ipebps_async + hfi_cmd_ipe_bps_map (one region) */
struct hfi_cmd_ipebps_mem_map {
	u32 size, pkt_type, opcode;
	u64 user_data1, user_data2;
	u32 num_fw_handles;
	u32 fw_handles[1];
	u64 map_user_data;
	u32 mem_map_request_num;
	u32 start_addr, len;
} __packed;

static int icp_mem_map(struct m52xq_icp *icp, u32 fw_handle, u32 iova, u32 len, bool map)
{
	struct hfi_cmd_ipebps_mem_map m = {};
	u32 buf[64];
	int ret;

	m.size = sizeof(m);
	m.pkt_type = HFI_CMD_IPEBPS_ASYNC_COMMAND_DIRECT;
	m.opcode = map ? HFI_IPEBPS_CMD_OPCODE_MEM_MAP : HFI_IPEBPS_CMD_OPCODE_MEM_UNMAP;
	m.num_fw_handles = 1;
	m.fw_handles[0] = fw_handle;
	m.mem_map_request_num = 1;
	m.start_addr = iova;
	m.len = len;
	ret = icp_write_cmd(icp, (u32 *)&m);
	if (!ret)
		ret = icp_wait_msg(icp, HFI_MSG_IPEBPS_ASYNC_COMMAND_DIRECT_ACK, buf, ARRAY_SIZE(buf), 3000);
	icp_log(icp, "MEM_%sMAP 0x%x+0x%x: %s opcode 0x%x err 0x%x\n", map ? "" : "UN", iova, len, ret > 0 ? "ack" : "no ack",
		ret > 2 ? buf[2] : 0, ret > 7 ? buf[7] : 0);
	return ret > 7 && !buf[7] ? 0 : -EIO;
}

/* type 1 = BPS, 2/3 = IPE: CONFIG_IO with the cfgbuf contents (iobuf_used bytes at IOVA ICP_IOBUF_IOVA) */
static int icp_cfgio_test(struct m52xq_icp *icp, u32 type)
{
	struct hfi_cmd_ipebps_indirect c = {};
	u32 buf[64], fw_handle, i;
	int ret, n;

	if (!icp->running)
		return -ENODEV;
	ret = icp_create_handle(icp, type, &fw_handle);
	if (ret)
		goto out;
	ret = icp_mem_map(icp, fw_handle, ICP_IOBUF_IOVA, ICP_IOBUF_LEN, true);
	if (ret) {
		icp_destroy_handle(icp, type, fw_handle);
		goto out;
	}
	c.size = sizeof(c);
	c.pkt_type = HFI_CMD_IPEBPS_ASYNC_COMMAND_INDIRECT;
	c.opcode = type == 1 ? HFI_IPEBPS_CMD_OPCODE_BPS_CONFIG_IO : HFI_IPEBPS_CMD_OPCODE_IPE_CONFIG_IO;
	c.user_data1 = 0x43464730ULL;	/* "CFG0" */
	c.num_fw_handles = 1;
	c.fw_handles[0] = fw_handle;
	c.payload_iova = ICP_IOBUF_IOVA;
	wmb();
	ret = icp_write_cmd(icp, (u32 *)&c);
	if (!ret)
		ret = icp_wait_msg(icp, HFI_MSG_IPEBPS_ASYNC_COMMAND_INDIRECT_ACK, buf, ARRAY_SIZE(buf), 3000);
	n = ret;
	/* hfi_msg_ipebps_async_ack: size, pkt_type, opcode, user_data1 (2), user_data2 (2), err_type, msg_data */
	icp_log(icp, "CONFIG_IO (%zu bytes) type %u: %s opcode 0x%x err 0x%x, %d words:", icp->iobuf_used, type,
		n > 0 ? "ack" : "no ack", n > 2 ? buf[2] : 0, n > 7 ? buf[7] : 0, n);
	for (i = 7; n > 0 && i < (u32)n && i < 16; i++)
		icp_log(icp, " %08x", buf[i]);
	icp_log(icp, "\n");
	icp_mem_map(icp, fw_handle, ICP_IOBUF_IOVA, ICP_IOBUF_LEN, false);
	icp_destroy_handle(icp, type, fw_handle);
	ret = n > 0 ? 0 : (n ? n : -EIO);
out:
	icp_dump_dbgq(icp);
	return ret;
}

static ssize_t cfgbuf_write(struct file *f, const char __user *ubuf, size_t len, loff_t *pos)
{
	struct m52xq_icp *icp = f->private_data;

	if (*pos + len > ICP_IOBUF_LEN)
		return -ENOSPC;
	mutex_lock(&icp->lock);
	if (!*pos)
		memset(icp->iobuf_kva, 0, ICP_IOBUF_LEN);
	if (copy_from_user(icp->iobuf_kva + *pos, ubuf, len)) {
		mutex_unlock(&icp->lock);
		return -EFAULT;
	}
	*pos += len;
	icp->iobuf_used = *pos;
	mutex_unlock(&icp->lock);
	return len;
}

static ssize_t cfgio_write(struct file *f, const char __user *ubuf, size_t len, loff_t *pos)
{
	struct m52xq_icp *icp = f->private_data;
	char c;
	int ret;

	if (len < 1 || get_user(c, ubuf) || c < '1' || c > '3')
		return -EINVAL;
	mutex_lock(&icp->lock);
	ret = icp_cfgio_test(icp, c - '0');
	mutex_unlock(&icp->lock);
	return ret ? ret : len;
}

static const struct file_operations cfgbuf_fops = {
	.open = simple_open,
	.write = cfgbuf_write,
};

static const struct file_operations cfgio_fops = {
	.open = simple_open,
	.write = cfgio_write,
};

static ssize_t session_write(struct file *f, const char __user *ubuf, size_t len, loff_t *pos)
{
	struct m52xq_icp *icp = f->private_data;
	char c;
	int ret;

	if (len < 1 || get_user(c, ubuf) || c < '1' || c > '3')
		return -EINVAL;
	mutex_lock(&icp->lock);
	ret = icp_session_test(icp, c - '0');
	mutex_unlock(&icp->lock);
	return ret ? ret : len;
}

static const struct file_operations session_fops = {
	.open = simple_open,
	.write = session_write,
};

static ssize_t ping_write(struct file *f, const char __user *ubuf, size_t len, loff_t *pos)
{
	struct m52xq_icp *icp = f->private_data;
	int ret;

	mutex_lock(&icp->lock);
	ret = icp_ping(icp);
	mutex_unlock(&icp->lock);
	return ret ? ret : len;
}

/* drain the firmware debug queue: struct hfi_msg_debug {size, pkt_type, msg_type, msg_size, ts_hi, ts_lo, text} */
static ssize_t dbgq_read(struct file *f, char __user *ubuf, size_t len, loff_t *pos)
{
	struct m52xq_icp *icp = f->private_data;
	size_t cap = 64 * 1024, used = 0;
	u32 *pkt;
	char *out;
	ssize_t r;
	int n;

	if (*pos)
		return 0;
	out = kvmalloc(cap, GFP_KERNEL);
	pkt = kvmalloc(25600 * 4, GFP_KERNEL);
	if (!out || !pkt) {
		kvfree(out);
		kvfree(pkt);
		return -ENOMEM;
	}
	mutex_lock(&icp->lock);
	if (icp->host_kva) {
		while (used < cap - 512 && (n = icp_read_q(icp, Q_DBG, pkt, 25600)) > 0) {
			u32 tsz = n > 6 ? (n - 6) * 4 : 0;
			u64 ts = ((u64)pkt[4] << 32) | pkt[5];

			if (pkt[3] && pkt[3] < tsz)
				tsz = pkt[3];
			used += scnprintf(out + used, cap - used, "[%llu us] type 0x%x/%u: %.*s\n", ts,
					  pkt[1], pkt[2], (int)tsz, (char *)&pkt[6]);
		}
		if (n < 0)
			used += scnprintf(out + used, cap - used, "(dbg queue read error %d)\n", n);
	}
	mutex_unlock(&icp->lock);
	r = simple_read_from_buffer(ubuf, len, pos, out, used);
	kvfree(out);
	kvfree(pkt);
	return r;
}

static const struct file_operations ping_fops = {
	.open = simple_open,
	.write = ping_write,
};

static const struct file_operations dbgq_fops = {
	.open = simple_open,
	.read = dbgq_read,
};

static ssize_t boot_write(struct file *f, const char __user *ubuf, size_t len, loff_t *pos)
{
	struct m52xq_icp *icp = f->private_data;
	char c;
	int ret;

	if (len < 1 || get_user(c, ubuf))
		return -EINVAL;
	mutex_lock(&icp->lock);
	if (c == '1')
		ret = icp_boot(icp);
	else {
		icp_stop(icp);
		ret = icp_power(icp, false);
		icp_log(icp, "stopped\n");
	}
	mutex_unlock(&icp->lock);
	return ret ? ret : len;
}

static ssize_t status_read(struct file *f, char __user *ubuf, size_t len, loff_t *pos)
{
	struct m52xq_icp *icp = f->private_data;

	return simple_read_from_buffer(ubuf, len, pos, icp->log, icp->log_len);
}

static const struct file_operations boot_fops = {
	.open = simple_open,
	.write = boot_write,
};

static const struct file_operations status_fops = {
	.open = simple_open,
	.read = status_read,
};

static int icp_open(struct inode *inode, struct file *f)
{
	struct m52xq_icp *icp = container_of(f->private_data, struct m52xq_icp, misc);
	int ret = 0;

	mutex_lock(&icp->lock);
	if (!icp->users && !icp->running)
		ret = icp_boot(icp);
	if (!ret)
		icp->users++;
	mutex_unlock(&icp->lock);
	return ret;
}

static int icp_release(struct inode *inode, struct file *f)
{
	struct m52xq_icp *icp = container_of(f->private_data, struct m52xq_icp, misc);

	mutex_lock(&icp->lock);
	if (!--icp->users) {
		icp_stop(icp);
		icp_power(icp, false);
	}
	mutex_unlock(&icp->lock);
	return 0;
}

static const struct file_operations icp_fops = {
	.owner = THIS_MODULE,
	.open = icp_open,
	.release = icp_release,
};

static void icp_unregister_memdev(void *data)
{
	platform_device_unregister(data);
}

static void icp_free_mem(void *data)
{
	struct m52xq_icp *icp = data;

	if (icp->host_kva)
		dma_free_coherent(icp->memdev, ICP_HOSTMEM_LEN, icp->host_kva, icp->host_phys);
	if (icp->secheap_kva)
		dma_free_coherent(icp->memdev, ICP_SECHEAP_LEN, icp->secheap_kva, icp->secheap_phys);
	if (icp->qdss_kva)
		dma_free_coherent(icp->memdev, ICP_QDSS_LEN, icp->qdss_kva, icp->qdss_phys);
	if (icp->iobuf_kva)
		dma_free_coherent(icp->memdev, ICP_IOBUF_LEN, icp->iobuf_kva, icp->iobuf_phys);
}

static int m52xq_icp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct platform_device_info pinfo = {
		.parent = dev,
		.name = "m52xq-icp-mem",
		.id = PLATFORM_DEVID_NONE,
		.dma_mask = DMA_BIT_MASK(32),
	};
	struct platform_device *memdev;
	struct reserved_mem *rmem;
	struct device_node *np;
	struct m52xq_icp *icp;
	int ret;

	icp = devm_kzalloc(dev, sizeof(*icp), GFP_KERNEL);
	if (!icp)
		return -ENOMEM;
	icp->dev = dev;
	mutex_init(&icp->lock);
	init_waitqueue_head(&icp->wq);

	icp->csr = devm_platform_ioremap_resource_byname(pdev, "sierra");
	if (IS_ERR(icp->csr))
		return PTR_ERR(icp->csr);

	icp->num_clks = devm_clk_bulk_get_all(dev, &icp->clks);
	if (icp->num_clks < 0)
		return dev_err_probe(dev, icp->num_clks, "clocks\n");
	for (ret = 0; ret < icp->num_clks; ret++) {
		unsigned long rate = 0;

		if (!strcmp(icp->clks[ret].id, "icp"))
			rate = icp_rate;
		else if (!strcmp(icp->clks[ret].id, "bps"))
			rate = bps_rate;
		else if (!strcmp(icp->clks[ret].id, "ipe"))
			rate = ipe_rate;
		if (rate)
			clk_set_rate(icp->clks[ret].clk, clk_round_rate(icp->clks[ret].clk, rate));
	}

	icp->icc = devm_of_icc_get(dev, "icp");
	if (IS_ERR(icp->icc))
		return dev_err_probe(dev, PTR_ERR(icp->icc), "interconnect\n");
	icp->icc_sf = devm_of_icc_get(dev, "sf");
	if (IS_ERR(icp->icc_sf))
		icp->icc_sf = NULL;

	/* BPS and IPE must be powered before SYS_INIT: the firmware resets their CDMs (downstream device_init) */
	if (of_count_phandle_with_args(dev->of_node, "power-domains", "#power-domain-cells") > 1) {
		static const char * const pd_names[] = { "top", "bps", "ipe" };
		const struct dev_pm_domain_attach_data pd_data = {
			.pd_names = pd_names,
			.num_pd_names = ARRAY_SIZE(pd_names),
		};

		ret = devm_pm_domain_attach_list(dev, &pd_data, &icp->pd_list);
		if (ret < 0)
			return dev_err_probe(dev, ret, "power domains\n");
	}

	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	rmem = np ? of_reserved_mem_lookup(np) : NULL;
	of_node_put(np);
	if (!rmem)
		return dev_err_probe(dev, -EINVAL, "no firmware memory-region\n");
	icp->fw_phys = rmem->base;
	icp->fw_size = rmem->size;
	icp->fw_kva = devm_memremap(dev, icp->fw_phys, icp->fw_size, MEMREMAP_WC);
	if (IS_ERR(icp->fw_kva))
		return PTR_ERR(icp->fw_kva);

	memdev = platform_device_register_full(&pinfo);
	if (IS_ERR(memdev))
		return PTR_ERR(memdev);
	icp->memdev = &memdev->dev;
	ret = devm_add_action_or_reset(dev, icp_unregister_memdev, memdev);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, icp_free_mem, icp);
	if (ret)
		return ret;
	icp->host_kva = dma_alloc_coherent(icp->memdev, ICP_HOSTMEM_LEN, &icp->host_phys, GFP_KERNEL);
	icp->secheap_kva = dma_alloc_coherent(icp->memdev, ICP_SECHEAP_LEN, &icp->secheap_phys,
					      GFP_KERNEL);
	icp->qdss_kva = dma_alloc_coherent(icp->memdev, ICP_QDSS_LEN, &icp->qdss_phys, GFP_KERNEL);
	icp->iobuf_kva = dma_alloc_coherent(icp->memdev, ICP_IOBUF_LEN, &icp->iobuf_phys, GFP_KERNEL);
	if (!icp->host_kva || !icp->secheap_kva || !icp->qdss_kva || !icp->iobuf_kva)
		return dev_err_probe(dev, -ENOMEM, "host memory\n");

	icp->domain = iommu_paging_domain_alloc(dev);
	if (IS_ERR(icp->domain))
		return dev_err_probe(dev, PTR_ERR(icp->domain), "iommu domain\n");
	ret = iommu_attach_device(icp->domain, dev);
	if (ret) {
		iommu_domain_free(icp->domain);
		return dev_err_probe(dev, ret, "iommu attach\n");
	}

	icp->irq = platform_get_irq(pdev, 0);
	if (icp->irq < 0)
		return icp->irq;
	ret = devm_request_irq(dev, icp->irq, icp_irq, IRQF_SHARED, "m52xq-icp", icp);
	if (ret)
		return ret;

	pm_runtime_enable(dev);
	platform_set_drvdata(pdev, icp);

	icp->misc.minor = MISC_DYNAMIC_MINOR;
	icp->misc.name = "m52xq-icp";
	icp->misc.fops = &icp_fops;
	ret = misc_register(&icp->misc);
	if (ret)
		return ret;

	icp->dbg = debugfs_create_dir("m52xq-icp", NULL);
	debugfs_create_file("boot", 0200, icp->dbg, icp, &boot_fops);
	debugfs_create_file("status", 0400, icp->dbg, icp, &status_fops);
	debugfs_create_file("ping", 0200, icp->dbg, icp, &ping_fops);
	debugfs_create_file("session", 0200, icp->dbg, icp, &session_fops);
	debugfs_create_file("cfgbuf", 0200, icp->dbg, icp, &cfgbuf_fops);
	debugfs_create_file("cfgio", 0200, icp->dbg, icp, &cfgio_fops);
	debugfs_create_file("dbgq", 0400, icp->dbg, icp, &dbgq_fops);

	dev_info(dev, "ICP A5: fw carveout %pa+0x%zx, host mem %pad, %d clocks, %u power domains; not started\n",
		 &icp->fw_phys, icp->fw_size, &icp->host_phys, icp->num_clks,
		 icp->pd_list ? icp->pd_list->num_pds : 1);
	return 0;
}

static void m52xq_icp_remove(struct platform_device *pdev)
{
	struct m52xq_icp *icp = platform_get_drvdata(pdev);

	debugfs_remove_recursive(icp->dbg);
	misc_deregister(&icp->misc);
	mutex_lock(&icp->lock);
	icp_stop(icp);
	icp_power(icp, false);
	mutex_unlock(&icp->lock);
	pm_runtime_disable(&pdev->dev);
	iommu_detach_device(icp->domain, &pdev->dev);
	iommu_domain_free(icp->domain);
}

static const struct of_device_id m52xq_icp_of_match[] = {
	{ .compatible = "samsung,m52xq-icp-a5" },
	{ }
};
MODULE_DEVICE_TABLE(of, m52xq_icp_of_match);

static struct platform_driver m52xq_icp_driver = {
	.probe = m52xq_icp_probe,
	.remove = m52xq_icp_remove,
	.driver = {
		.name = "m52xq-icp",
		.of_match_table = m52xq_icp_of_match,
	},
};
module_platform_driver(m52xq_icp_driver);

MODULE_DESCRIPTION("M52XQ camera ICP (A5) bring-up");
MODULE_LICENSE("GPL");
