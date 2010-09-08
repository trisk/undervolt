/*
 * Copyright (c) 2010 Albert Lee <trisk@forkgnu.org>.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 * 
 * The above copyright notice and this permission notice shall be 
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR 
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef __KERNEL__
#define __KERNEL__
#endif

#ifndef MODULE
#define MODULE
#endif

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/mutex.h>
#include <asm/uaccess.h>

#define UNDERVOLT_NAME "undervolt"
#define UNDERVOLT_DESCRIPTION "Undervolt driver for HTC/Qualcomm devices"
#define UNDERVOLT_VERSION "1.0a"

MODULE_VERSION(UNDERVOLT_VERSION);
MODULE_LICENSE("Dual MIT/GPL");
MODULE_ALIAS(UNDERVOLT_NAME);
MODULE_DESCRIPTION(UNDERVOLT_DESCRIPTION);
MODULE_AUTHOR("Albert Lee <trisk@forkgnu.org>");

struct clkctl_acpu_speed {
	unsigned acpu_khz;
	unsigned clk_cfg;
	unsigned clk_sel;
	unsigned sc_l_value;
	unsigned lpj;
	int      vdd;
	unsigned axiclk_khz;
};

#define CPUFREQ_ENTRY_INVALID ~0
#define CPUFREQ_TABLE_END     ~1
struct cpufreq_frequency_table {
	unsigned index;
	unsigned frequency;
};

struct uv_map {
	unsigned acpu_khz;
	int      vdd;
	int      uv_vdd;
};

struct oc_map {
	unsigned frequency;
	unsigned oc_frequency;
};

struct m_search_pat {
	unsigned word;
	unsigned mask;
};

extern int regulator_set_voltage(void *, int, int);
extern void cpufreq_frequency_table_get_attr(void *, int);

#define PROC_NAME UNDERVOLT_NAME
static struct proc_dir_entry *proc_entry = NULL;

static char read_buf[16];

#define UV_ENTRY_MAX 31
static struct uv_map undervolt_tbl[UV_ENTRY_MAX + 1];
static DEFINE_MUTEX(uv_mutex);

static int vdd_undervolt = 0;

static struct clkctl_acpu_speed *acpu_freq_tbl = NULL;
static struct cpufreq_frequency_table *freq_table = NULL;

#if 0
/*
  b4:   e3a0301c        mov     r3, #28 ; 0x1c
  b8:   e3a01000        mov     r1, #0  ; 0x0
  bc:   e0020293        mul     r2, r3, r2
  c0:   e59f3040        ldr     r3, [pc, #64]   ; 108
  c4:   e7831002        str     r1, [r3, r2]
*/

static struct m_search_pat fixup_pat[] = {
	{ 0xe3a0001c, 0xffff0fff }, /* e3a0?01c */
	{ 0xe3a00000, 0xffff0fff }, /* e3a0?000 */
	{ 0xe0000090, 0xfff0f0f0 }, /* e00?0?9? */
	{ 0xe59f0000, 0xffff0000 }, /* e59f???? */
	{ 0 },
};

static unsigned long fixup_addr = 0;
static unsigned long fixup_size = DEFAULT_FIXUP_SIZE;
#endif

static unsigned long acpuclk_set_rate_addr = 0;

#define DEFAULT_ASR_DEPTH (16 * sizeof(unsigned))
static unsigned long acpuclk_set_rate_depth = DEFAULT_ASR_DEPTH;

#define DEFAULT_CFTGA_DEPTH (2 * sizeof(unsigned))
static unsigned long cpufreq_frequency_table_get_attr_depth =
	DEFAULT_CFTGA_DEPTH;

#define DEFAULT_RSV_DEPTH (48 * sizeof(unsigned))
static unsigned long regulator_set_voltage_depth = DEFAULT_RSV_DEPTH;

/*
 27c:   e59f41d0        ldr     r4, [pc, #464]  ; 454 <acpuclk_set_rate+0x204>
 280:   e5942000        ldr     r2, [r4]
*/

static struct m_search_pat acpuclk_set_rate_pat[] = {
	{ 0xe59f0000, 0xffff0000 },
	{ 0xe5900000, 0xfff00fff },
	{ 0 },
};


static struct m_search_pat cpufreq_frequency_table_get_attr_pat[] = {
	{ 0xe59f0000, 0xffff0000 },
	{ 0xe5800000, 0xfff0ffff },
	{ 0 },
};

/*
    2028:       e1570001        cmp     r7, r1
    202c:       b1a07001        movlt   r7, r1
    2030:       e1580002        cmp     r8, r2
    2034:       b1a02008        movlt   r2, r8
    2038:       a1a02002        movge   r2, r2
*/

static struct m_search_pat regulator_set_voltage_pat[] = {
	{ 0xe1500000, 0xfff0fff0 },
	{ 0xb1a00000, 0xffff0ff0 },
	{ 0xe1500000, 0xfff0fff0 },
	{ 0xb1a00000, 0xffff0ff0 },
	{ 0xa1a00000, 0xffff0ff0 },
	{ 0 },
};

static int __init undervolt_init(void);
static void __exit undervolt_exit(void);
static int undervolt_read(char *page, char **start, off_t off, int count,
                          int *eof, void *data);
static int undervolt_write(struct file *filp, const char __user *buf,
                               unsigned long len, void *data);
static unsigned *m_search(struct m_search_pat *n, unsigned *hp, unsigned *l);
static struct clkctl_acpu_speed *get_freq_tbl(struct m_search_pat *pat,
                                              void *addr, unsigned long depth);
static struct cpufreq_frequency_table *get_freq_table(struct m_search_pat *pat,
                                                      void *addr,
                                                      unsigned long depth);
static int patch_reg_set_voltage(struct m_search_pat *pat, void *addr,
                                 unsigned long depth);
static int init_uv_tbl(struct uv_map *uv_tbl,
                       struct clkctl_acpu_speed *freq_tbl);
static int update_uv_tbl(struct uv_map *uv_tbl, int vdd, int uv_vdd);
static void update_uv_tbl_adjust(struct uv_map *uv_tbl, int adj);
static int update_freq_tbl(struct clkctl_acpu_speed *freq_tbl,
                           struct uv_map *uv_tbl, int enable);

module_param(acpuclk_set_rate_addr, long, S_IRUGO);
MODULE_PARM_DESC(acpuclk_set_rate_addr, "acpuclk_set_rate function address");
module_param(acpuclk_set_rate_depth, long, S_IRUGO);
MODULE_PARM_DESC(acpuclk_set_rate_depth,
                 "acpuclk_set_rate function search depth");
module_param(cpufreq_frequency_table_get_attr_depth, long, S_IRUGO);
MODULE_PARM_DESC(cpufreq_frequency_table_get_attr_depth,
                 "cpufreq_frequency_table_get_attr function search depth");
module_param(regulator_set_voltage_depth, long, S_IRUGO);
MODULE_PARM_DESC(regulator_set_voltage_depth,
                 "regulator_set_voltage function search depth");
module_param(vdd_undervolt, int, S_IRUGO);
MODULE_PARM_DESC(vdd_undervolt, "mV to decrease all voltages by");

module_init(undervolt_init);
module_exit(undervolt_exit);

int undervolt_init(void)
{
	printk(KERN_INFO "%s: %s %s\n", UNDERVOLT_NAME, UNDERVOLT_DESCRIPTION,
	       UNDERVOLT_VERSION);
	printk(KERN_INFO "%s: (c) 2010 Albert Lee <trisk@forkgnu.org>\n",
	       UNDERVOLT_NAME);

	if (acpuclk_set_rate_addr == 0) {
		printk(KERN_ERR "%s: Missing required parameter " \
		       "`acpuclk_set_rate_addr'\n",
		       __func__);
		return -EINVAL;
	}

	proc_entry = create_proc_entry(PROC_NAME, S_IFREG | S_IWUSR | S_IRUGO,
	                               NULL);
	if (proc_entry == NULL) {
		printk(KERN_ERR "%s: Unable to create /proc/%s\n",
		       __func__, PROC_NAME);
		return -ENOMEM;
	}
	proc_entry->read_proc = undervolt_read;
	proc_entry->write_proc = undervolt_write;

	acpu_freq_tbl = get_freq_tbl(acpuclk_set_rate_pat,
	                             (void *)acpuclk_set_rate_addr,
	                             acpuclk_set_rate_depth);
	if (acpu_freq_tbl == NULL) {
		printk(KERN_ERR "%s: acpu_freq_tbl not found\n", __func__);
		undervolt_exit();
		return -ENXIO;
	}

	freq_table = get_freq_table(cpufreq_frequency_table_get_attr_pat,
	                            (void *)cpufreq_frequency_table_get_attr,
	                            cpufreq_frequency_table_get_attr_depth);
	if (freq_table == NULL) {
		printk(KERN_INFO "%s: freq_table for cpufreq not found, " \
		       "overclocking will not be possible\n", UNDERVOLT_NAME);
	}

	if (patch_reg_set_voltage(regulator_set_voltage_pat,
	                          (void *)regulator_set_voltage,
	                          regulator_set_voltage_depth) != 0) {
		printk(KERN_INFO "%s: Unable to patch regulator_set_voltage, " \
		       "voltage will be subject to board constraints\n",
		       UNDERVOLT_NAME);
	}

	init_uv_tbl(undervolt_tbl, acpu_freq_tbl);

	if (vdd_undervolt > 0) {
		update_uv_tbl_adjust(undervolt_tbl, -1 * vdd_undervolt);
		update_freq_tbl(acpu_freq_tbl, undervolt_tbl, 1);
	}

	return 0;
}

void undervolt_exit(void)
{
	if (acpu_freq_tbl != NULL) {
		update_freq_tbl(acpu_freq_tbl, undervolt_tbl, 0);
	}

	if (proc_entry) {
		remove_proc_entry(PROC_NAME, NULL);
		proc_entry = NULL;
	}

	return;
}

int undervolt_read(char *page, char **start, off_t off, int count, int *eof,
                   void *data)
{
	int len = 0;
	
	struct uv_map *uv_tbl = undervolt_tbl;

	if ((count == 0) || (off > 0)) {
		return 0;
	}

	mutex_lock(&uv_mutex);
	for (; uv_tbl->acpu_khz != 0; uv_tbl++) {
		len += snprintf(page + len, count - len, "%4d %4d %8u\n",
		                uv_tbl->vdd, uv_tbl->uv_vdd, uv_tbl->acpu_khz);
		if (len >= count) {
			len = count;
			break;
		}
	}
	mutex_unlock(&uv_mutex);
	*eof = (len > count) ? 0 : 1;

	return len;
}

int undervolt_write(struct file *filp, const char __user *buf,
                    unsigned long len, void *data)
{
	int i = 0, j;
	int vdd[2] = { 0, 0 };
	int sign = 0;

	if (len > sizeof(read_buf)) {
		return -ENOSPC;
	}

	if (len == 0) {
		return 0;
	}

	if (copy_from_user(read_buf, buf, len) != 0) {
		return -EINVAL;
	}

	if ((len == 1) || (read_buf[1] == '\n')) {
		switch (read_buf[0]) {
		case '0':
		case '1':
			i = read_buf[0] - '0';
			mutex_lock(&uv_mutex);
			update_freq_tbl(acpu_freq_tbl, undervolt_tbl, i);
			if (i == 0) {
				init_uv_tbl(undervolt_tbl, acpu_freq_tbl);
			}
			mutex_unlock(&uv_mutex);
			printk(KERN_INFO "%s: acpu_freq_tbl updated\n",
			       UNDERVOLT_NAME);
		default:
			return len;
		}
	}

	if (read_buf[0] == '-') {
		sign = -1;
		i++;
	} else if (read_buf[0] == '+') {
		sign = 1;
		i++;
	}

	for (j = 0; i < len; i++) {
		char c = read_buf[i];
		if ((c >= '0') && (c <= '9')) {
			vdd[j] *= 10;
			vdd[j] += (c - '0');
		} else if ((c == ' ') || (c == '\t')) {
			if (vdd[j] != 0) {
				j++;
				if ((sign != 0) || (j > 1)) {
					break;
				}
			}
		} else {
			break;
		}
	}

	if (sign != 0) {
		mutex_lock(&uv_mutex);
		update_uv_tbl_adjust(undervolt_tbl, sign * vdd[0]);
		mutex_unlock(&uv_mutex);
	} else if (vdd[0] > 0 && vdd[1] > 0) {
		mutex_lock(&uv_mutex);
		update_uv_tbl(undervolt_tbl, vdd[0], vdd[1]);
		mutex_unlock(&uv_mutex);
	}

	return len;
}

unsigned *m_search(struct m_search_pat *n, unsigned *hp, unsigned *l)
{
	struct m_search_pat *np = n;

	for (; hp < l; hp++) {
#ifdef DEBUG
		printk(KERN_DEBUG "%s: %p: %08x\n", __func__, hp, *hp); 
#endif
		if ((*hp & np->mask) == np->word) {
#ifdef DEBUG
			printk(KERN_DEBUG "%s:  matches: %08x\n", __func__,
			       *hp & np->mask);
#endif
			np++;

			if ((np->word == 0) && (np->mask == 0)) {
				/* Return beginning of matched sequence */
				np--;
				return hp - (np - n);
			}
		} else {
			np = n;
		}
	}

	return NULL;
}

/*
enum {
	SRC_RAW,
	SRC_SCPLL,
	SRC_AXI,
	SRC_PLL1,
};

static const unsigned match[] = {
	19200,  0, SRC_RAW, 0, (unsigned)-1, 1050, 14000,
	128000, 0, SRC_AXI, 0, (unsigned)-1, 1050, 14000,
};
*/

struct clkctl_acpu_speed *get_freq_tbl(struct m_search_pat *pat, void *addr,
                                       unsigned long depth)
{
	unsigned *insn;
	void **tblp;
	struct clkctl_acpu_speed *tbl;

	insn = m_search(pat, addr, addr + depth);
	if (insn == NULL) {
		return NULL;
	}
	
	/* e59f?XXX ldr */
	tblp = (void **)((unsigned)insn + (*insn & 0xfff) + 0x8);
	tbl = (struct clkctl_acpu_speed *)(*tblp);
	if ((tbl->acpu_khz != 19200) || (tbl->axiclk_khz != 14000)) {
		return NULL;
	}

	return tbl;
}

struct cpufreq_frequency_table *get_freq_table(struct m_search_pat *pat,
                                               void *addr,
                                               unsigned long depth)
{
	unsigned *insn;
	void **tblp;
	struct cpufreq_frequency_table *tbl;

	insn = m_search(pat, addr, addr + depth);
	if (insn == NULL) {
		return NULL;
	}
	
	/* e59f?XXX ldr */
	tblp = (void **)((unsigned)insn + (*insn & 0xfff) + 0x8);
	tbl = (struct cpufreq_frequency_table *)(*tblp);
	if ((tbl->index != 0) || (tbl->frequency != CPUFREQ_ENTRY_INVALID)) {
		return NULL;
	}

	return tbl;
}

int patch_reg_set_voltage(struct m_search_pat *pat, void *addr,
                          unsigned long depth)
{
	unsigned *insn;

#ifdef DEBUG
	printk(KERN_DEBUG "%s: regulator_set_voltage: 0x%p\n", __func__, addr);
#endif
	insn = m_search(pat, addr, addr + depth);
	if (insn == NULL) {
		return -1;
	}
	do {
		switch (*insn & 0xfff00000) {
		case 0xa1a00000: /* movgt */
		case 0xb1a00000: /* movlt */
		case 0xe1500000: /* cmp  */
			*insn = 0xe1500000; /* cmp r0, r0 */
			insn++;
			break;
		default:
			goto done;
		}
	} while (--depth);
done:
	return 0;
}

int init_uv_tbl(struct uv_map *uv_tbl, struct clkctl_acpu_speed *freq_tbl)
{
	struct uv_map *uv_tbl_end = &uv_tbl[UV_ENTRY_MAX];

	for (; freq_tbl->acpu_khz != 0; freq_tbl++) {
		if (uv_tbl == uv_tbl_end) {
			break;
		}
		uv_tbl->acpu_khz = freq_tbl->acpu_khz;
		uv_tbl->vdd = uv_tbl->uv_vdd = freq_tbl->vdd;
		uv_tbl++;
	}
	uv_tbl->acpu_khz = 0;

	return 0;
}

int update_uv_tbl(struct uv_map *uv_tbl, int vdd, int uv_vdd)
{
	if ((vdd == 0) || (uv_vdd == 0)) {
		return -1;
	}

	for (; uv_tbl->acpu_khz != 0; uv_tbl++) {
		if (uv_tbl->vdd == vdd) {
			uv_tbl->uv_vdd = uv_vdd;
		} else if (uv_tbl->vdd > vdd) {
			break;
		}
	}

	return 0;
}

int update_freq_tbl(struct clkctl_acpu_speed *freq_tbl, struct uv_map *uv_tbl,
                    int enable)
{
	for (; freq_tbl->acpu_khz != 0; freq_tbl++) {
		for (; uv_tbl->acpu_khz != 0; uv_tbl++) {
			if (uv_tbl->acpu_khz == freq_tbl->acpu_khz) {
				if (enable) {
					freq_tbl->vdd = uv_tbl->uv_vdd;
				} else {
					freq_tbl->vdd = uv_tbl->vdd;
				}
				break;
			}
		}
	}

	return 0;
}

void update_uv_tbl_adjust(struct uv_map *uv_tbl, int adj)
{
	for (; uv_tbl->acpu_khz != 0; uv_tbl++) {
		uv_tbl->uv_vdd += adj;
	}

	return;
}
