// SPDX-License-Identifier: GPL-2.0
/*
 * Intel Speed Select -- Enumerate and control features
 * Copyright (c) 2019 Intel Corporation.
 */

#include <ctype.h>
#include <linux/isst_if.h>

#include "isst.h"

struct process_cmd_struct {
	char *feature;
	char *command;
	void (*process_fn)(int arg);
	int arg;
};

static const char *version_str = "v1.23";

static const int supported_api_ver = 3;
static struct isst_if_platform_info isst_platform_info;
static char *progname;
static int debug_flag;
static FILE *outf;

static int cpu_model;
static int cpu_stepping;
static int extended_family;

#define MAX_CPUS_IN_ONE_REQ 512
static short max_target_cpus;
static unsigned short target_cpus[MAX_CPUS_IN_ONE_REQ];

static int topo_max_cpus;
static size_t present_cpumask_size;
static cpu_set_t *present_cpumask;
static size_t target_cpumask_size;
static cpu_set_t *target_cpumask;
static int tdp_level = 0xFF;
static int fact_bucket = 0xFF;
static int fact_avx = 0xFF;
static unsigned long long fact_trl;
static int out_format_json;
static int cmd_help;
static int force_online_offline;
static int auto_mode;
static int fact_enable_fail;
static int cgroupv2;
static int max_pkg_id;
static int max_die_id;
static int max_die_id_package_0;

/* clos related */
static int current_clos = -1;
static int clos_epp = -1;
static int clos_prop_prio = -1;
static int clos_min = -1;
static int clos_max = -1;
static int clos_desired = -1;
static int clos_priority_type;
static int cpu_0_cgroupv2;
static int cpu_0_workaround(int isolate);

struct _cpu_map {
	unsigned short core_id;
	unsigned short pkg_id;
	unsigned short die_id;
	unsigned short punit_id;
	unsigned short punit_cpu;
	unsigned short punit_cpu_core;
	unsigned short initialized;
};
struct _cpu_map *cpu_map;

struct cpu_topology {
	short cpu;
	short core_id;
	short pkg_id;
	short die_id;
};

FILE *get_output_file(void)
{
	return outf;
}

int is_debug_enabled(void)
{
	return debug_flag;
}

void debug_printf(const char *format, ...)
{
	va_list args;

	va_start(args, format);

	if (debug_flag)
		vprintf(format, args);

	va_end(args);
}


int is_clx_n_platform(void)
{
	if (cpu_model == 0x55)
		if (cpu_stepping == 0x6 || cpu_stepping == 0x7)
			return 1;
	return 0;
}

int is_skx_based_platform(void)
{
	if (cpu_model == 0x55)
		return 1;

	return 0;
}

int is_spr_platform(void)
{
	if (cpu_model == 0x8F)
		return 1;

	return 0;
}

int is_emr_platform(void)
{
	if (cpu_model == 0xCF)
		return 1;

	return 0;
}


int is_icx_platform(void)
{
	if (cpu_model == 0x6A || cpu_model == 0x6C)
		return 1;

	return 0;
}

static int is_dmr_plus_platform(void)
{
	if (extended_family == 0x04)
		return 1;

	return 0;
}

static int update_cpu_model(void)
{
	unsigned int ebx, ecx, edx;
	unsigned int fms, family;

	__cpuid(1, fms, ebx, ecx, edx);
	family = (fms >> 8) & 0xf;
	extended_family = (fms >> 20) & 0x0f;
	cpu_model = (fms >> 4) & 0xf;
	if (family == 6 || family == 0xf)
		cpu_model += ((fms >> 16) & 0xf) << 4;

	cpu_stepping = fms & 0xf;
	/* only three CascadeLake-N models are supported */
	if (is_clx_n_platform()) {
		FILE *fp;
		size_t n = 0;
		char *line = NULL;
		int ret = 1;

		fp = fopen("/proc/cpuinfo", "r");
		if (!fp)
			err(-1, "cannot open /proc/cpuinfo\n");

		while (getline(&line, &n, fp) > 0) {
			if (strstr(line, "model name")) {
				if (strstr(line, "6252N") ||
				    strstr(line, "6230N") ||
				    strstr(line, "5218N"))
					ret = 0;
				break;
			}
		}
		free(line);
		fclose(fp);
		return ret;
	}
	return 0;
}

int api_version(void)
{
        return isst_platform_info.api_version;
}

/* Open a file, and exit on failure */
static FILE *fopen_or_exit(const char *path, const char *mode)
{
	FILE *filep = fopen(path, mode);

	if (!filep)
		err(1, "%s: open failed", path);

	return filep;
}

/* Parse a file containing a single int */
static int parse_int_file(int fatal, const char *fmt, ...)
{
	va_list args;
	char path[PATH_MAX];
	FILE *filep;
	int value;

	va_start(args, fmt);
	vsnprintf(path, sizeof(path), fmt, args);
	va_end(args);
	if (fatal) {
		filep = fopen_or_exit(path, "r");
	} else {
		filep = fopen(path, "r");
		if (!filep)
			return -1;
	}
	if (fscanf(filep, "%d", &value) != 1)
		err(1, "%s: failed to parse number from file", path);
	fclose(filep);

	return value;
}

int cpufreq_sysfs_present(void)
{
	DIR *dir;

	dir = opendir("/sys/devices/system/cpu/cpu0/cpufreq");
	if (dir) {
		closedir(dir);
		return 1;
	}

	return 0;
}

int out_format_is_json(void)
{
	return out_format_json;
}

static int get_stored_topology_info(int cpu, int *core_id, int *pkg_id, int *die_id)
{
	const char *pathname = "/var/run/isst_cpu_topology.dat";
	struct cpu_topology cpu_top;
	FILE *fp;
	int ret;

	fp = fopen(pathname, "rb");
	if (!fp)
		return -1;

	ret = fseek(fp, cpu * sizeof(cpu_top), SEEK_SET);
	if (ret)
		goto err_ret;

	ret = fread(&cpu_top, sizeof(cpu_top), 1, fp);
	if (ret != 1) {
		ret = -1;
		goto err_ret;
	}

	*pkg_id = cpu_top.pkg_id;
	*core_id = cpu_top.core_id;
	*die_id = cpu_top.die_id;
	ret = 0;

err_ret:
	fclose(fp);

	return ret;
}

static void store_cpu_topology(void)
{
	const char *pathname = "/var/run/isst_cpu_topology.dat";
	FILE *fp;
	int i;

	fp = fopen(pathname, "rb");
	if (fp) {
		/* Mapping already exists */
		fclose(fp);
		return;
	}

	fp = fopen(pathname, "wb");
	if (!fp) {
		fprintf(stderr, "Can't create file:%s\n", pathname);
		return;
	}

	fprintf(stderr, "Caching topology information\n");

	for (i = 0; i < topo_max_cpus; ++i) {
		struct cpu_topology cpu_top;

		cpu_top.core_id = parse_int_file(0,
			"/sys/devices/system/cpu/cpu%d/topology/core_id", i);
		if (cpu_top.core_id < 0)
			cpu_top.core_id = -1;

		cpu_top.pkg_id = parse_int_file(0,
			"/sys/devices/system/cpu/cpu%d/topology/physical_package_id", i);
		if (cpu_top.pkg_id < 0)
			cpu_top.pkg_id = -1;

		cpu_top.die_id = parse_int_file(0,
			"/sys/devices/system/cpu/cpu%d/topology/die_id", i);
		if (cpu_top.die_id < 0)
			cpu_top.die_id = -1;

		cpu_top.cpu = i;

		if (fwrite(&cpu_top, sizeof(cpu_top), 1, fp) != 1) {
			fprintf(stderr, "Can't write to:%s\n", pathname);
			break;
		}
	}

	fclose(fp);
}

static int get_physical_package_id(int cpu)
{
	int ret;

	if (cpu < 0)
		return -1;

	if (cpu_map && cpu_map[cpu].initialized)
		return cpu_map[cpu].pkg_id;

	ret = parse_int_file(0,
			"/sys/devices/system/cpu/cpu%d/topology/physical_package_id",
			cpu);
	if (ret < 0) {
		int core_id, pkg_id, die_id;

		ret = get_stored_topology_info(cpu, &core_id, &pkg_id, &die_id);
		if (!ret)
			return pkg_id;
	}

	return ret;
}

static int get_physical_core_id(int cpu)
{
	int ret;

	if (cpu < 0)
		return -1;

	if (cpu_map && cpu_map[cpu].initialized)
		return cpu_map[cpu].core_id;

	ret = parse_int_file(0,
			"/sys/devices/system/cpu/cpu%d/topology/core_id",
			cpu);
	if (ret < 0) {
		int core_id, pkg_id, die_id;

		ret = get_stored_topology_info(cpu, &core_id, &pkg_id, &die_id);
		if (!ret)
			return core_id;
	}

	return ret;
}

static int get_physical_die_id(int cpu)
{
	int ret;

	if (cpu < 0)
		return -1;

	if (cpu_map && cpu_map[cpu].initialized)
		return cpu_map[cpu].die_id;

	ret = parse_int_file(0,
			"/sys/devices/system/cpu/cpu%d/topology/die_id",
			cpu);
	if (ret < 0) {
		int core_id, pkg_id, die_id;

		ret = get_stored_topology_info(cpu, &core_id, &pkg_id, &die_id);
		if (!ret) {
			if (die_id < 0)
				die_id = 0;

			return die_id;
		}
	}

	if (ret < 0)
		ret = 0;

	return ret;
}

static int get_physical_punit_id(int cpu)
{
	if (cpu < 0)
		return -1;

	if (cpu_map && cpu_map[cpu].initialized)
		return cpu_map[cpu].punit_id;

	return -1;
}

void set_isst_id(struct isst_id *id, int cpu)
{
	id->cpu = cpu;

	id->pkg = get_physical_package_id(cpu);
	if (id->pkg >= MAX_PACKAGE_COUNT)
		id->pkg = -1;

	id->die = get_physical_die_id(cpu);
	if (id->die >= MAX_DIE_PER_PACKAGE)
		id->die = -1;

	id->punit = get_physical_punit_id(cpu);
	if (id->punit >= MAX_PUNIT_PER_DIE)
		id->punit = -1;
}

int is_cpu_in_power_domain(int cpu, struct isst_id *id)
{
	struct isst_id tid;

	set_isst_id(&tid, cpu);

	if (id->pkg == tid.pkg && id->die == tid.die && id->punit == tid.punit)
		return 1;

	return 0;
}

int get_cpufreq_base_freq(int cpu)
{
	return parse_int_file(0, "/sys/devices/system/cpu/cpu%d/cpufreq/base_frequency", cpu);
}

int get_topo_max_cpus(void)
{
	return topo_max_cpus;
}

static unsigned int is_cpu_online(int cpu)
{
	char buffer[128];
	int fd, ret;
	unsigned char online;

	snprintf(buffer, sizeof(buffer),
		 "/sys/devices/system/cpu/cpu%d/online", cpu);

	fd = open(buffer, O_RDONLY);
	if (fd < 0)
		return fd;

	ret = read(fd, &online, sizeof(online));
	close(fd);

	if (ret == -1)
		return ret;

	if (online == '1')
		online = 1;
	else
		online = 0;

	return online;
}

void set_cpu_online_offline(int cpu, int state)
{
	char buffer[128];
	int fd, ret;

	if (cpu_0_cgroupv2 && !cpu) {
		fprintf(stderr, "Will use cgroup v2 for CPU 0\n");
		cpu_0_workaround(!state);
		return;
	}

	snprintf(buffer, sizeof(buffer),
		 "/sys/devices/system/cpu/cpu%d/online", cpu);

	fd = open(buffer, O_WRONLY);
	if (fd < 0) {
		if (!cpu) {
			fprintf(stderr, "This system is not configured for CPU 0 online/offline\n");
			fprintf(stderr, "Will use cgroup v2\n");
			cpu_0_workaround(!state);
			return;
		}
		err(-1, "%s open failed", buffer);
	}

	if (state)
		ret = write(fd, "1\n", 2);
	else
		ret = write(fd, "0\n", 2);

	if (ret == -1)
		perror("Online/Offline: Operation failed\n");

	close(fd);
}

static void force_all_cpus_online(void)
{
	int i;

	fprintf(stderr, "Forcing all CPUs online\n");

	for (i = 0; i < topo_max_cpus; ++i)
		set_cpu_online_offline(i, 1);

	unlink("/var/run/isst_cpu_topology.dat");
}

void for_each_online_power_domain_in_set(void (*callback)(struct isst_id *, void *, void *,
						     void *, void *),
				    void *arg1, void *arg2, void *arg3,
				    void *arg4)
{
	struct isst_id id;
	int cpus[MAX_PACKAGE_COUNT][MAX_DIE_PER_PACKAGE][MAX_PUNIT_PER_DIE];
	int valid_mask[MAX_PACKAGE_COUNT][MAX_DIE_PER_PACKAGE] = {0};
	int i, j, k;

	memset(cpus, -1, sizeof(cpus));

	for (i = 0; i < topo_max_cpus; ++i) {
		int online;

		if (!CPU_ISSET_S(i, present_cpumask_size, present_cpumask))
			continue;

		online = parse_int_file(
			i != 0, "/sys/devices/system/cpu/cpu%d/online", i);
		if (online < 0)
			online = 1; /* online entry for CPU 0 needs some special configs */

		if (!online)
			continue;

		set_isst_id(&id, i);

		if (id.pkg < 0 || id.die < 0 || id.punit < 0)
			continue;

		id.die = id.die % (max_die_id_package_0 + 1);

		valid_mask[id.pkg][id.die] = 1;

		if (cpus[id.pkg][id.die][id.punit] == -1)
			cpus[id.pkg][id.die][id.punit] = i;
	}

	for (i = 0; i < MAX_PACKAGE_COUNT; i++) {
		if (max_die_id > max_pkg_id) {
			for (k = 0; k < MAX_PUNIT_PER_DIE && k < MAX_DIE_PER_PACKAGE; k++) {
				id.cpu = cpus[i][k][k];
				id.pkg = i;
				id.die = get_physical_die_id(id.cpu);
				id.punit = k;
				if (isst_is_punit_valid(&id))
					callback(&id, arg1, arg2, arg3, arg4);
			}
			continue;
		}

		for (j = 0; j < MAX_DIE_PER_PACKAGE; j++) {
			/*
			 * Fix me:
			 * How to check a non-cpu die for a package/die with all cpu offlined?
			 */
			if (!valid_mask[i][j])
				continue;
			for (k = 0; k < MAX_PUNIT_PER_DIE; k++) {
				id.cpu = cpus[i][j][k];
				id.pkg = i;
				if (id.cpu >= 0)
					id.die = get_physical_die_id(id.cpu);
				else
					id.die = id.pkg;
				id.punit = k;
				if (isst_is_punit_valid(&id))
					callback(&id, arg1, arg2, arg3, arg4);
			}
		}
	}
}

static void for_each_online_target_cpu_in_set(
	void (*callback)(struct isst_id *, void *, void *, void *, void *), void *arg1,
	void *arg2, void *arg3, void *arg4)
{
	int i, found = 0;
	struct isst_id id;

	for (i = 0; i < topo_max_cpus; ++i) {
		int online;

		if (!CPU_ISSET_S(i, target_cpumask_size, target_cpumask))
			continue;
		if (i)
			online = parse_int_file(
				1, "/sys/devices/system/cpu/cpu%d/online", i);
		else
			online =
				1; /* online entry for CPU 0 needs some special configs */

		set_isst_id(&id, i);
		if (online && callback) {
			callback(&id, arg1, arg2, arg3, arg4);
			found = 1;
		}
	}

	if (!found)
		fprintf(stderr, "No valid CPU in the list\n");
}

#define BITMASK_SIZE 32
static void set_max_cpu_num(void)
{
	FILE *filep;
	unsigned long dummy;
	int i;

	topo_max_cpus = 0;
	for (i = 0; i < 256; ++i) {
		char path[256];

		snprintf(path, sizeof(path),
			 "/sys/devices/system/cpu/cpu%d/topology/thread_siblings", i);
		filep = fopen(path, "r");
		if (filep)
			break;
	}

	if (!filep) {
		fprintf(stderr, "Can't get max cpu number\n");
		exit(0);
	}

	while (fscanf(filep, "%lx,", &dummy) == 1)
		topo_max_cpus += BITMASK_SIZE;
	fclose(filep);

	debug_printf("max cpus %d\n", topo_max_cpus);
}

size_t alloc_cpu_set(cpu_set_t **cpu_set)
{
	cpu_set_t *_cpu_set;
	size_t size;

	_cpu_set = CPU_ALLOC((topo_max_cpus + 1));
	if (_cpu_set == NULL)
		err(3, "CPU_ALLOC");
	size = CPU_ALLOC_SIZE((topo_max_cpus + 1));
	CPU_ZERO_S(size, _cpu_set);

	*cpu_set = _cpu_set;
	return size;
}

void free_cpu_set(cpu_set_t *cpu_set)
{
	CPU_FREE(cpu_set);
}

static int cpu_cnt[MAX_PACKAGE_COUNT][MAX_DIE_PER_PACKAGE][MAX_PUNIT_PER_DIE];

int get_max_punit_core_id(struct isst_id *id)
{
	int max_id = 0;
	int i;

	for (i = 0; i < topo_max_cpus; ++i)
	{
		if (!CPU_ISSET_S(i, present_cpumask_size, present_cpumask))
			continue;

		if (is_cpu_in_power_domain(i, id) &&
		    cpu_map[i].punit_cpu_core > max_id)
			max_id = cpu_map[i].punit_cpu_core;
	}

	return max_id;
}

int get_cpu_count(struct isst_id *id)
{
	if (id->pkg < 0 || id->die < 0 || id->punit < 0)
		return 0;

	return cpu_cnt[id->pkg][id->die][id->punit];
}

static void update_punit_cpu_info(__u32 physical_cpu, struct _cpu_map *cpu_map)
{
	if (api_version() > 1) {
		/*
		 * MSR 0x54 format
		 *	[15:11] PM_DOMAIN_ID
		 *	[10:3] MODULE_ID (aka IDI_AGENT_ID)
		 *	[2:0] LP_ID (We don't care about these bits we only
		 *		care die and core id
		 *	For Atom:
		 *	[2] Always 0
		 *	[1:0] core ID within module
		 *	For Core
		 *	[2:1] Always 0
		 *	[0] thread ID
		 */
		cpu_map->punit_id = (physical_cpu >> 11) & 0x1f;
		cpu_map->punit_cpu_core = (physical_cpu >> 3) & 0xff;
		cpu_map->punit_cpu = physical_cpu & 0x7ff;
	} else {
		int punit_id;

		/*
		 * MSR 0x53 format
		 * Format
		 *      Bit 0 – thread ID
		 *      Bit 8:1 – core ID
		 *      Bit 13:9 – punit ID
		 */
		cpu_map->punit_cpu = physical_cpu & 0x1ff;
		cpu_map->punit_cpu_core = (cpu_map->punit_cpu >> 1); // shift to get core id
		punit_id = (physical_cpu >> 9) & 0x1f;

		if (punit_id >= MAX_PUNIT_PER_DIE)
			punit_id = 0;

		cpu_map->punit_id = punit_id;
	}
}

static void create_cpu_map(void)
{
	const char *pathname = "/dev/isst_interface";
	size_t size;
	DIR *dir;
	int i, fd = 0;
	struct isst_if_cpu_maps map;

	/* Use calloc to make sure the memory is initialized to Zero */
	cpu_map = calloc(topo_max_cpus, sizeof(*cpu_map));
	if (!cpu_map)
		err(3, "cpumap");

	fd = open(pathname, O_RDWR);
	if (fd < 0 && !is_clx_n_platform())
		err(-1, "%s open failed", pathname);

	size = alloc_cpu_set(&present_cpumask);
	present_cpumask_size = size;

	for (i = 0; i < topo_max_cpus; ++i) {
		char buffer[256];
		int pkg_id, die_id, core_id, punit_id;

		/* check if CPU is online */
		snprintf(buffer, sizeof(buffer),
			 "/sys/devices/system/cpu/cpu%d", i);
		dir = opendir(buffer);
		if (!dir)
			continue;
		closedir(dir);

		CPU_SET_S(i, size, present_cpumask);

		pkg_id = get_physical_package_id(i);
		die_id = get_physical_die_id(i);
		core_id = get_physical_core_id(i);

		if (pkg_id < 0 || die_id < 0 || core_id < 0)
			continue;

		cpu_map[i].pkg_id = pkg_id;
		cpu_map[i].die_id = die_id;
		cpu_map[i].core_id = core_id;

		if (max_pkg_id < pkg_id)
			max_pkg_id = pkg_id;

		punit_id = 0;

		if (fd >= 0) {
			map.cmd_count = 1;
			map.cpu_map[0].logical_cpu = i;
			debug_printf(" map logical_cpu:%d\n",
				     map.cpu_map[0].logical_cpu);
			if (ioctl(fd, ISST_IF_GET_PHY_ID, &map) == -1) {
				perror("ISST_IF_GET_PHY_ID");
				fprintf(outf, "Error: map logical_cpu:%d\n",
					map.cpu_map[0].logical_cpu);
			} else {
				update_punit_cpu_info(map.cpu_map[0].physical_cpu, &cpu_map[i]);
				punit_id = cpu_map[i].punit_id;
			}
		}
		cpu_map[i].initialized = 1;

		cpu_cnt[pkg_id][die_id][punit_id]++;

		if (max_die_id < die_id)
			max_die_id = die_id;

		if (!pkg_id && max_die_id_package_0 < die_id)
			max_die_id_package_0 = die_id;

		debug_printf(
			"map logical_cpu:%d core: %d die:%d pkg:%d punit:%d punit_cpu:%d punit_core:%d\n",
			i, cpu_map[i].core_id, cpu_map[i].die_id,
			cpu_map[i].pkg_id, cpu_map[i].punit_id,
			cpu_map[i].punit_cpu, cpu_map[i].punit_cpu_core);
	}
	if (fd >= 0)
		close(fd);

	size = alloc_cpu_set(&target_cpumask);
	target_cpumask_size = size;
	for (i = 0; i < max_target_cpus; ++i) {
		if (!CPU_ISSET_S(target_cpus[i], present_cpumask_size,
				 present_cpumask))
			continue;

		CPU_SET_S(target_cpus[i], size, target_cpumask);
	}
}

void set_cpu_mask_from_punit_coremask(struct isst_id *id, unsigned long long core_mask,
				      size_t core_cpumask_size,
				      cpu_set_t *core_cpumask, int *cpu_cnt)
{
	int i, cnt = 0;

	if (id->cpu < 0)
		return;

	*cpu_cnt = 0;

	for (i = 0; i < 64; ++i) {
		if (core_mask & BIT_ULL(i)) {
			int j;

			for (j = 0; j < topo_max_cpus; ++j) {
				if (!CPU_ISSET_S(j, present_cpumask_size, present_cpumask))
					continue;

				if (is_cpu_in_power_domain(j, id) &&
				    cpu_map[j].punit_cpu_core == i) {
					CPU_SET_S(j, core_cpumask_size,
						  core_cpumask);
					++cnt;
				}
			}
		}
	}

	*cpu_cnt = cnt;
}

int find_phy_core_num(int logical_cpu)
{
	if (logical_cpu < topo_max_cpus)
		return cpu_map[logical_cpu].punit_cpu_core;

	return -EINVAL;
}

int use_cgroupv2(void)
{
	return cgroupv2;
}

int enable_cpuset_controller(void)
{
	int fd, ret;

	fd = open("/sys/fs/cgroup/cgroup.subtree_control", O_RDWR, 0);
	if (fd < 0) {
		debug_printf("Can't activate cpuset controller\n");
		debug_printf("Either you are not root user or CGroup v2 is not supported\n");
		return fd;
	}

	ret = write(fd, " +cpuset", strlen(" +cpuset"));
	close(fd);

	if (ret == -1) {
		debug_printf("Can't activate cpuset controller: Write failed\n");
		return ret;
	}

	return 0;
}

int isolate_cpus(struct isst_id *id, int mask_size, cpu_set_t *cpu_mask, int level, int cpu_0_only)
{
	int i, first, curr_index, index, ret, fd;
	static char str[512], dir_name[64];
	static char cpuset_cpus[128];
	int str_len = sizeof(str);
	DIR *dir;

	snprintf(dir_name, sizeof(dir_name), "/sys/fs/cgroup/%d-%d-%d", id->pkg, id->die, id->punit);
	dir = opendir(dir_name);
	if (!dir) {
		ret = mkdir(dir_name, 0744);
		if (ret) {
			debug_printf("Can't create dir:%s errno:%d\n", dir_name, errno);
			return ret;
		}
	}
	closedir(dir);

	if (!level) {
		sprintf(cpuset_cpus, "%s/cpuset.cpus.partition", dir_name);

		fd = open(cpuset_cpus, O_RDWR, 0);
		if (fd < 0) {
			return fd;
		}

		ret = write(fd, "member", strlen("member"));
		if (ret == -1) {
			printf("Can't update to member\n");
			return ret;
		}

		return 0;
	}

	if (!CPU_COUNT_S(mask_size, cpu_mask)) {
		return -1;
	}

	curr_index = 0;
	first = 1;
	str[0] = '\0';

	if (cpu_0_only) {
		snprintf(str, str_len, "0");
		goto create_partition;
	}

	for (i = 0; i < get_topo_max_cpus(); ++i) {
		if (!is_cpu_in_power_domain(i, id))
			continue;

		if (CPU_ISSET_S(i, mask_size, cpu_mask))
			continue;

		if (!first) {
			index = snprintf(&str[curr_index],
					 str_len - curr_index, ",");
			curr_index += index;
			if (curr_index >= str_len)
				break;
		}
		index = snprintf(&str[curr_index], str_len - curr_index, "%d",
				 i);
		curr_index += index;
		if (curr_index >= str_len)
			break;
		first = 0;
	}

create_partition:
	debug_printf("isolated CPUs list: package:%d curr_index:%d [%s]\n", id->pkg, curr_index ,str);

	snprintf(cpuset_cpus, sizeof(cpuset_cpus), "%s/cpuset.cpus", dir_name);

	fd = open(cpuset_cpus, O_RDWR, 0);
	if (fd < 0) {
		return fd;
	}

	ret = write(fd, str, strlen(str));
	close(fd);

	if (ret == -1) {
		debug_printf("Can't activate cpuset controller: Write failed\n");
		return ret;
	}

	snprintf(cpuset_cpus, sizeof(cpuset_cpus), "%s/cpuset.cpus.partition", dir_name);

	fd = open(cpuset_cpus, O_RDWR, 0);
	if (fd < 0) {
		return fd;
	}

	ret = write(fd, "isolated", strlen("isolated"));
	if (ret == -1) {
		debug_printf("Can't update to isolated\n");
		ret = write(fd, "root", strlen("root"));
		if (ret == -1)
			debug_printf("Can't update to root\n");
	}

	close(fd);

	if (ret < 0)
		return ret;

	return 0;
}

static int cpu_0_workaround(int isolate)
{
	int fd, fd1, len, ret;
	cpu_set_t cpu_mask;
	struct isst_id id;
	char str[2];

	debug_printf("isolate CPU 0 state: %d\n", isolate);

	if (isolate)
		goto isolate;

	/* First check if CPU 0 was isolated to remove isolation. */

	/* If the cpuset.cpus doesn't exist, that means that none of the CPUs are isolated*/
	fd = open("/sys/fs/cgroup/0-0-0/cpuset.cpus", O_RDONLY, 0);
	if (fd < 0)
		return 0;

	len = read(fd, str, sizeof(str));
	/* Error check, but unlikely to fail. If fails that means that not isolated */
	if (len == -1)
		return 0;


	/* Is CPU 0 is in isolate list, the display is sorted so first element will be CPU 0*/
	if (str[0] != '0') {
		close(fd);
		return 0;
	}

	fd1 = open("/sys/fs/cgroup/0-0-0/cpuset.cpus.partition", O_RDONLY, 0);
	/* Unlikely that, this attribute is not present, but handle error */
	if (fd1 < 0) {
		close(fd);
		return 0;
	}

	/* Is CPU 0 already changed partition to "member" */
	len = read(fd1, str, sizeof(str));
	if (len != -1 && str[0] == 'm') {
		close(fd1);
		close(fd);
		return 0;
	}

	close(fd1);
	close(fd);

	debug_printf("CPU 0 was isolated before, so remove isolation\n");

isolate:
	ret = enable_cpuset_controller();
	if (ret)
		goto isolate_fail;

	CPU_ZERO(&cpu_mask);
	memset(&id, 0, sizeof(struct isst_id));
	CPU_SET(0, &cpu_mask);

	ret = isolate_cpus(&id, sizeof(cpu_mask), &cpu_mask, isolate, 1);
isolate_fail:
	if (ret)
		fprintf(stderr, "Can't isolate CPU 0\n");

	return ret;
}

static int isst_fill_platform_info(void)
{
	const char *pathname = "/dev/isst_interface";
	int fd;

	if (is_clx_n_platform()) {
		isst_platform_info.api_version = 1;
		goto set_platform_ops;
	}

	fd = open(pathname, O_RDWR);
	if (fd < 0)
		err(-1, "%s open failed", pathname);

	if (ioctl(fd, ISST_IF_GET_PLATFORM_INFO, &isst_platform_info) == -1) {
		perror("ISST_IF_GET_PLATFORM_INFO");
		close(fd);
		return -1;
	}

	close(fd);

	if (isst_platform_info.api_version > supported_api_ver) {
		printf("Incompatible API versions; Upgrade of tool is required\n");
		return -1;
	}

set_platform_ops:
	if (isst_set_platform_ops(isst_platform_info.api_version)) {
		fprintf(stderr, "Failed to set platform callbacks\n");
		exit(0);
	}
	return 0;
}

void get_isst_status(struct isst_id *id, void *arg1, void *arg2, void *arg3, void *arg4)
{
	struct isst_pkg_ctdp pkg_dev;
	struct isst_id *tid = (struct isst_id *)arg2;
	int *mask = (int *)arg3;
	int *max_level = (int *)arg4;
	int j, ret;

	/* Only check the first cpu power domain */
	if (id->cpu < 0 || tid->cpu >= 0)
		return;

	ret = isst_get_ctdp_levels(id, &pkg_dev);
	if (ret)
		return;

	if (pkg_dev.enabled)
		*mask |= BIT(0);

	if (pkg_dev.locked)
		*mask |= BIT(1);

	if (*max_level < pkg_dev.levels)
		*max_level = pkg_dev.levels;

	for (j = 0; j <= pkg_dev.levels; ++j) {
		struct isst_pkg_ctdp_level_info ctdp_level;

		ret = isst_get_ctdp_control(id, j, &ctdp_level);
		if (ret)
			continue;

		if (ctdp_level.fact_support)
			*mask |= BIT(2);

		if (ctdp_level.pbf_support)
			*mask |= BIT(3);
	}

	tid->cpu = id->cpu;
	tid->pkg = id->pkg;
	tid->die = id->die;
	tid->punit = id->punit;
}

static void isst_print_extended_platform_info(void)
{
	int cp_state, cp_cap;
	struct isst_id id;
	int mask = 0, max_level = 0;

	id.cpu = -1;
	for_each_online_power_domain_in_set(get_isst_status, NULL, &id, &mask, &max_level);

	if (mask & BIT(0)) {
		fprintf(outf, "Intel(R) SST-PP (feature perf-profile) is supported\n");
	} else {
		fprintf(outf, "Intel(R) SST-PP (feature perf-profile) is not supported\n");
		fprintf(outf, "Only performance level 0 (base level) is present\n");
	}

	if (mask & BIT(1))
		fprintf(outf, "TDP level change control is locked\n");
	else
		fprintf(outf, "TDP level change control is unlocked, max level: %d\n", max_level);

	if (mask & BIT(2))
		fprintf(outf, "Intel(R) SST-TF (feature turbo-freq) is supported\n");
	else
		fprintf(outf, "Intel(R) SST-TF (feature turbo-freq) is not supported\n");

	if (mask & BIT(3))
		fprintf(outf, "Intel(R) SST-BF (feature base-freq) is supported\n");
	else
		fprintf(outf, "Intel(R) SST-BF (feature base-freq) is not supported\n");

	if (isst_read_pm_config(&id, &cp_state, &cp_cap)) {
		fprintf(outf, "Intel(R) SST-CP (feature core-power) status is unknown\n");
		return;
	}

	if (cp_cap)
		fprintf(outf, "Intel(R) SST-CP (feature core-power) is supported\n");
	else
		fprintf(outf, "Intel(R) SST-CP (feature core-power) is not supported\n");
}

static void isst_print_platform_information(void)
{
	if (is_clx_n_platform()) {
		fprintf(stderr, "\nThis option in not supported on this platform\n");
		exit(0);
	}

	/* Early initialization to create working cpu_map */
	set_max_cpu_num();
	create_cpu_map();

	fprintf(outf, "Platform: API version : %d\n",
		isst_platform_info.api_version);
	fprintf(outf, "Platform: Driver version : %d\n",
		isst_platform_info.driver_version);
	fprintf(outf, "Platform: mbox supported : %d\n",
		isst_platform_info.mbox_supported);
	fprintf(outf, "Platform: mmio supported : %d\n",
		isst_platform_info.mmio_supported);
	isst_print_extended_platform_info();

	exit(0);
}

static char *local_str0, *local_str1;
static void exec_on_get_ctdp_cpu(struct isst_id *id, void *arg1, void *arg2, void *arg3,
				 void *arg4)
{
	int (*fn_ptr)(struct isst_id *id, void *arg);
	int ret;

	fn_ptr = arg1;
	ret = fn_ptr(id, arg2);
	if (ret)
		isst_display_error_info_message(1, "get_tdp_* failed", 0, 0);
	else
		isst_ctdp_display_core_info(id, outf, arg3,
					    *(unsigned int *)arg4,
					    local_str0, local_str1);
}

#define _get_tdp_level(desc, suffix, object, help, str0, str1)			\
	static void get_tdp_##object(int arg)                                    \
	{                                                                         \
		struct isst_pkg_ctdp ctdp;                                        \
\
		if (cmd_help) {                                                   \
			fprintf(stderr,                                           \
				"Print %s [No command arguments are required]\n", \
				help);                                            \
			exit(0);                                                  \
		}                                                                 \
		local_str0 = str0;						  \
		local_str1 = str1;						  \
		isst_ctdp_display_information_start(outf);                        \
		if (max_target_cpus)                                              \
			for_each_online_target_cpu_in_set(                        \
				exec_on_get_ctdp_cpu, isst_get_ctdp_##suffix,     \
				&ctdp, desc, &ctdp.object);                       \
		else                                                              \
			for_each_online_power_domain_in_set(exec_on_get_ctdp_cpu,      \
						       isst_get_ctdp_##suffix,    \
						       &ctdp, desc,               \
						       &ctdp.object);             \
		isst_ctdp_display_information_end(outf);                          \
	}

_get_tdp_level("get-config-levels", levels, levels, "Max TDP level", NULL, NULL);
_get_tdp_level("get-config-version", levels, version, "TDP version", NULL, NULL);
_get_tdp_level("get-config-enabled", levels, enabled, "perf-profile enable status", "disabled", "enabled");
_get_tdp_level("get-config-current_level", levels, current_level,
	       "Current TDP Level", NULL, NULL);
_get_tdp_level("get-lock-status", levels, locked, "TDP lock status", "unlocked", "locked");

struct isst_pkg_ctdp clx_n_pkg_dev;

static int clx_n_get_base_ratio(void)
{
	FILE *fp;
	char *begin, *end, *line = NULL;
	char number[5];
	float value = 0;
	size_t n = 0;

	fp = fopen("/proc/cpuinfo", "r");
	if (!fp)
		err(-1, "cannot open /proc/cpuinfo\n");

	while (getline(&line, &n, fp) > 0) {
		if (strstr(line, "model name")) {
			/* this is true for CascadeLake-N */
			begin = strstr(line, "@ ") + 2;
			end = strstr(line, "GHz");
			strncpy(number, begin, end - begin);
			value = atof(number) * 10;
			break;
		}
	}
	free(line);
	fclose(fp);

	return (int)(value);
}

static int clx_n_config(struct isst_id *id)
{
	int i, ret;
	unsigned long cpu_bf;
	struct isst_pkg_ctdp_level_info *ctdp_level;
	struct isst_pbf_info *pbf_info;

	ctdp_level = &clx_n_pkg_dev.ctdp_level[0];
	pbf_info = &ctdp_level->pbf_info;
	ctdp_level->core_cpumask_size =
			alloc_cpu_set(&ctdp_level->core_cpumask);

	/* find the frequency base ratio */
	ctdp_level->tdp_ratio = clx_n_get_base_ratio();
	if (ctdp_level->tdp_ratio == 0) {
		debug_printf("CLX: cn base ratio is zero\n");
		ret = -1;
		goto error_ret;
	}

	/* find the high and low priority frequencies */
	pbf_info->p1_high = 0;
	pbf_info->p1_low = ~0;

	for (i = 0; i < topo_max_cpus; i++) {
		if (!CPU_ISSET_S(i, present_cpumask_size, present_cpumask))
			continue;

		if (!is_cpu_in_power_domain(i, id))
			continue;

		CPU_SET_S(i, ctdp_level->core_cpumask_size,
			  ctdp_level->core_cpumask);

		cpu_bf = parse_int_file(1,
			"/sys/devices/system/cpu/cpu%d/cpufreq/base_frequency",
					i);
		if (cpu_bf > pbf_info->p1_high)
			pbf_info->p1_high = cpu_bf;
		if (cpu_bf < pbf_info->p1_low)
			pbf_info->p1_low = cpu_bf;
	}

	if (pbf_info->p1_high == ~0UL) {
		debug_printf("CLX: maximum base frequency not set\n");
		ret = -1;
		goto error_ret;
	}

	if (pbf_info->p1_low == 0) {
		debug_printf("CLX: minimum base frequency not set\n");
		ret = -1;
		goto error_ret;
	}

	/* convert frequencies back to ratios */
	pbf_info->p1_high = pbf_info->p1_high / 100000;
	pbf_info->p1_low = pbf_info->p1_low / 100000;

	/* create high priority cpu mask */
	pbf_info->core_cpumask_size = alloc_cpu_set(&pbf_info->core_cpumask);
	for (i = 0; i < topo_max_cpus; i++) {
		if (!CPU_ISSET_S(i, present_cpumask_size, present_cpumask))
			continue;

		if (!is_cpu_in_power_domain(i, id))
			continue;

		cpu_bf = parse_int_file(1,
			"/sys/devices/system/cpu/cpu%d/cpufreq/base_frequency",
					i);
		cpu_bf = cpu_bf / 100000;
		if (cpu_bf == pbf_info->p1_high)
			CPU_SET_S(i, pbf_info->core_cpumask_size,
				  pbf_info->core_cpumask);
	}

	/* extra ctdp & pbf struct parameters */
	ctdp_level->processed = 1;
	ctdp_level->pbf_support = 1; /* PBF is always supported and enabled */
	ctdp_level->pbf_enabled = 1;
	ctdp_level->fact_support = 0; /* FACT is never supported */
	ctdp_level->fact_enabled = 0;

	return 0;

error_ret:
	free_cpu_set(ctdp_level->core_cpumask);
	return ret;
}

static void dump_clx_n_config_for_cpu(struct isst_id *id, void *arg1, void *arg2,
				   void *arg3, void *arg4)
{
	int ret;

	if (tdp_level != 0xff && tdp_level != 0) {
		isst_display_error_info_message(1, "Invalid level", 1, tdp_level);
		exit(0);
	}

	ret = clx_n_config(id);
	if (ret) {
		debug_printf("clx_n_config failed");
	} else {
		struct isst_pkg_ctdp_level_info *ctdp_level;
		struct isst_pbf_info *pbf_info;

		ctdp_level = &clx_n_pkg_dev.ctdp_level[0];
		pbf_info = &ctdp_level->pbf_info;
		clx_n_pkg_dev.processed = 1;
		isst_ctdp_display_information(id, outf, tdp_level, &clx_n_pkg_dev);
		free_cpu_set(ctdp_level->core_cpumask);
		free_cpu_set(pbf_info->core_cpumask);
	}
}

static void dump_isst_config_for_cpu(struct isst_id *id, void *arg1, void *arg2,
				     void *arg3, void *arg4)
{
	struct isst_pkg_ctdp pkg_dev;
	int ret;

	memset(&pkg_dev, 0, sizeof(pkg_dev));
	ret = isst_get_process_ctdp(id, tdp_level, &pkg_dev);
	if (ret) {
		isst_display_error_info_message(1, "Failed to get perf-profile info on cpu", 1, id->cpu);
		isst_ctdp_display_information_end(outf);
		exit(1);
	} else {
		isst_ctdp_display_information(id, outf, tdp_level, &pkg_dev);
		isst_get_process_ctdp_complete(id, &pkg_dev);
	}
}

static void dump_isst_config(int arg)
{
	void *fn;

	if (cmd_help) {
		fprintf(stderr,
			"Print Intel(R) Speed Select Technology Performance profile configuration\n");
		fprintf(stderr,
			"including base frequency and turbo frequency configurations\n");
		fprintf(stderr, "Optional: -l|--level : Specify tdp level\n");
		fprintf(stderr,
			"\tIf no arguments, dump information for all TDP levels\n");
		exit(0);
	}

	if (!is_clx_n_platform())
		fn = dump_isst_config_for_cpu;
	else
		fn = dump_clx_n_config_for_cpu;

	isst_ctdp_display_information_start(outf);

	if (max_target_cpus)
		for_each_online_target_cpu_in_set(fn, NULL, NULL, NULL, NULL);
	else
		for_each_online_power_domain_in_set(fn, NULL, NULL, NULL, NULL);

	isst_ctdp_display_information_end(outf);
}

static void adjust_scaling_max_from_base_freq(int cpu);

static void set_tdp_level_for_cpu(struct isst_id *id, void *arg1, void *arg2, void *arg3,
				  void *arg4)
{
	struct isst_pkg_ctdp pkg_dev;
	int ret;

	ret = isst_get_ctdp_levels(id, &pkg_dev);
	if (ret) {
		isst_display_error_info_message(1, "Get TDP level failed", 0, 0);
		isst_ctdp_display_information_end(outf);
		exit(1);
	}

	if (pkg_dev.current_level == tdp_level) {
		debug_printf("TDP level already set. Skipped\n");
		goto display_result;
	}

	ret = isst_set_tdp_level(id, tdp_level);
	if (ret) {
		isst_display_error_info_message(1, "Set TDP level failed", 0, 0);
		isst_ctdp_display_information_end(outf);
		exit(1);
	}

display_result:
	isst_display_result(id, outf, "perf-profile", "set_tdp_level", ret);
	if (force_online_offline && id->cpu >= 0) {
		struct isst_pkg_ctdp_level_info ctdp_level;

		/* Wait for updated base frequencies */
		usleep(2000);

		/* Adjusting uncore freq */
		if (!is_dmr_plus_platform())
			isst_adjust_uncore_freq(id, tdp_level, &ctdp_level);

		fprintf(stderr, "Option is set to online/offline\n");
		ctdp_level.core_cpumask_size =
			alloc_cpu_set(&ctdp_level.core_cpumask);
		ret = isst_get_coremask_info(id, tdp_level, &ctdp_level);
		if (ret) {
			isst_display_error_info_message(1, "Can't get coremask, online/offline option is ignored", 0, 0);
			goto free_mask;
		}

		if (use_cgroupv2()) {
			int ret;

			fprintf(stderr, "Using cgroup v2 in lieu of online/offline\n");
			ret = enable_cpuset_controller();
			if (ret)
				goto use_offline;

			ret = isolate_cpus(id, ctdp_level.core_cpumask_size,
					   ctdp_level.core_cpumask, tdp_level, 0);
			if (ret)
				goto use_offline;

			goto free_mask;
		}

use_offline:
		if (ctdp_level.cpu_count) {
			int i, max_cpus = get_topo_max_cpus();
			for (i = 0; i < max_cpus; ++i) {
				if (!is_cpu_in_power_domain(i, id))
					continue;
				if (CPU_ISSET_S(i, ctdp_level.core_cpumask_size, ctdp_level.core_cpumask)) {
					fprintf(stderr, "online cpu %d\n", i);
					set_cpu_online_offline(i, 1);
					adjust_scaling_max_from_base_freq(i);
				} else {
					fprintf(stderr, "offline cpu %d\n", i);
					set_cpu_online_offline(i, 0);
				}
			}
		}
free_mask:
		free_cpu_set(ctdp_level.core_cpumask);
	}
}

static void set_tdp_level(int arg)
{
	if (cmd_help) {
		fprintf(stderr, "Set Config TDP level\n");
		fprintf(stderr,
			"\t Arguments: -l|--level : Specify tdp level\n");
		fprintf(stderr,
			"\t Optional Arguments: -o | online : online/offline for the tdp level\n");
		fprintf(stderr,
			"\t  online/offline operation has limitations, refer to Linux hotplug documentation\n");
		exit(0);
	}

	if (tdp_level == 0xff) {
		isst_display_error_info_message(1, "Invalid command: specify tdp_level", 0, 0);
		exit(1);
	}
	isst_ctdp_display_information_start(outf);
	if (max_target_cpus)
		for_each_online_target_cpu_in_set(set_tdp_level_for_cpu, NULL,
						  NULL, NULL, NULL);
	else
		for_each_online_power_domain_in_set(set_tdp_level_for_cpu, NULL,
					       NULL, NULL, NULL);
	isst_ctdp_display_information_end(outf);
}

static void clx_n_dump_pbf_config_for_cpu(struct isst_id *id, void *arg1, void *arg2,
				       void *arg3, void *arg4)
{
	int ret;

	ret = clx_n_config(id);
	if (ret) {
		isst_display_error_info_message(1, "clx_n_config failed", 0, 0);
	} else {
		struct isst_pkg_ctdp_level_info *ctdp_level;
		struct isst_pbf_info *pbf_info;

		ctdp_level = &clx_n_pkg_dev.ctdp_level[0];
		pbf_info = &ctdp_level->pbf_info;
		isst_pbf_display_information(id, outf, tdp_level, pbf_info);
		free_cpu_set(ctdp_level->core_cpumask);
		free_cpu_set(pbf_info->core_cpumask);
	}
}

static void dump_pbf_config_for_cpu(struct isst_id *id, void *arg1, void *arg2, void *arg3,
				    void *arg4)
{
	struct isst_pbf_info pbf_info;
	int ret;

	ret = isst_get_pbf_info(id, tdp_level, &pbf_info);
	if (ret) {
		isst_display_error_info_message(1, "Failed to get base-freq info at this level", 1, tdp_level);
		isst_ctdp_display_information_end(outf);
		exit(1);
	} else {
		isst_pbf_display_information(id, outf, tdp_level, &pbf_info);
		free_cpu_set(pbf_info.core_cpumask);
	}
}

static void dump_pbf_config(int arg)
{
	void *fn;

	if (cmd_help) {
		fprintf(stderr,
			"Print Intel(R) Speed Select Technology base frequency configuration for a TDP level\n");
		fprintf(stderr,
			"\tArguments: -l|--level : Specify tdp level\n");
		exit(0);
	}

	if (tdp_level == 0xff) {
		isst_display_error_info_message(1, "Invalid command: specify tdp_level", 0, 0);
		exit(1);
	}

	if (!is_clx_n_platform())
		fn = dump_pbf_config_for_cpu;
	else
		fn = clx_n_dump_pbf_config_for_cpu;

	isst_ctdp_display_information_start(outf);

	if (max_target_cpus)
		for_each_online_target_cpu_in_set(fn, NULL, NULL, NULL, NULL);
	else
		for_each_online_power_domain_in_set(fn, NULL, NULL, NULL, NULL);

	isst_ctdp_display_information_end(outf);
}

static int set_clos_param(struct isst_id *id, int clos, int epp, int wt, int min, int max)
{
	struct isst_clos_config clos_config;
	int ret;

	ret = isst_pm_get_clos(id, clos, &clos_config);
	if (ret) {
		isst_display_error_info_message(1, "isst_pm_get_clos failed", 0, 0);
		return ret;
	}
	clos_config.clos_min = min;
	clos_config.clos_max = max;
	clos_config.epp = epp;
	clos_config.clos_prop_prio = wt;
	ret = isst_set_clos(id, clos, &clos_config);
	if (ret) {
		isst_display_error_info_message(1, "isst_set_clos failed", 0, 0);
		return ret;
	}

	return 0;
}

static int set_cpufreq_scaling_min_max(int cpu, int max, int freq)
{
	char buffer[128], freq_str[16];
	int fd, ret, len;

	if (max)
		snprintf(buffer, sizeof(buffer),
			 "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_max_freq", cpu);
	else
		snprintf(buffer, sizeof(buffer),
			 "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_min_freq", cpu);

	fd = open(buffer, O_WRONLY);
	if (fd < 0)
		return fd;

	snprintf(freq_str, sizeof(freq_str), "%d", freq);
	len = strlen(freq_str);
	ret = write(fd, freq_str, len);
	if (ret == -1) {
		close(fd);
		return ret;
	}
	close(fd);

	return 0;
}

static int no_turbo(void)
{
	return parse_int_file(0, "/sys/devices/system/cpu/intel_pstate/no_turbo");
}

static void adjust_scaling_max_from_base_freq(int cpu)
{
	int base_freq, scaling_max_freq;

	scaling_max_freq = parse_int_file(0, "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_max_freq", cpu);
	base_freq = get_cpufreq_base_freq(cpu);
	if (scaling_max_freq < base_freq || no_turbo())
		set_cpufreq_scaling_min_max(cpu, 1, base_freq);
}

static void adjust_scaling_min_from_base_freq(int cpu)
{
	int base_freq, scaling_min_freq;

	scaling_min_freq = parse_int_file(0, "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_min_freq", cpu);
	base_freq = get_cpufreq_base_freq(cpu);
	if (scaling_min_freq < base_freq)
		set_cpufreq_scaling_min_max(cpu, 0, base_freq);
}

static int set_clx_pbf_cpufreq_scaling_min_max(struct isst_id *id)
{
	struct isst_pkg_ctdp_level_info *ctdp_level;
	struct isst_pbf_info *pbf_info;
	int i, freq, freq_high, freq_low;
	int ret;

	ret = clx_n_config(id);
	if (ret) {
		debug_printf("cpufreq_scaling_min_max failed for CLX");
		return ret;
	}

	ctdp_level = &clx_n_pkg_dev.ctdp_level[0];
	pbf_info = &ctdp_level->pbf_info;
	freq_high = pbf_info->p1_high * 100000;
	freq_low = pbf_info->p1_low * 100000;

	for (i = 0; i < get_topo_max_cpus(); ++i) {
		if (!is_cpu_in_power_domain(i, id))
			continue;

		if (CPU_ISSET_S(i, pbf_info->core_cpumask_size,
				  pbf_info->core_cpumask))
			freq = freq_high;
		else
			freq = freq_low;

		set_cpufreq_scaling_min_max(i, 1, freq);
		set_cpufreq_scaling_min_max(i, 0, freq);
	}

	return 0;
}

static int set_cpufreq_scaling_min_max_from_cpuinfo(int cpu, int cpuinfo_max, int scaling_max)
{
	char buffer[128], min_freq[16];
	int fd, ret, len;

	if (!CPU_ISSET_S(cpu, present_cpumask_size, present_cpumask))
		return -1;

	if (cpuinfo_max)
		snprintf(buffer, sizeof(buffer),
			 "/sys/devices/system/cpu/cpu%d/cpufreq/cpuinfo_max_freq", cpu);
	else
		snprintf(buffer, sizeof(buffer),
			 "/sys/devices/system/cpu/cpu%d/cpufreq/cpuinfo_min_freq", cpu);

	fd = open(buffer, O_RDONLY);
	if (fd < 0)
		return fd;

	len = read(fd, min_freq, sizeof(min_freq));
	close(fd);

	if (len < 0)
		return len;

	if (scaling_max)
		snprintf(buffer, sizeof(buffer),
			 "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_max_freq", cpu);
	else
		snprintf(buffer, sizeof(buffer),
			 "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_min_freq", cpu);

	fd = open(buffer, O_WRONLY);
	if (fd < 0)
		return fd;

	min_freq[15] = '\0';
	len = strlen(min_freq);
	ret = write(fd, min_freq, len);
	if (ret == -1) {
		close(fd);
		return ret;
	}
	close(fd);

	return 0;
}

static void set_scaling_min_to_cpuinfo_max(struct isst_id *id)
{
	int i;

	if (id->cpu < 0)
		return;

	for (i = 0; i < get_topo_max_cpus(); ++i) {
		if (!is_cpu_in_power_domain(i, id))
			continue;

		if (is_cpu_online(i) != 1)
			continue;

		adjust_scaling_max_from_base_freq(i);
		set_cpufreq_scaling_min_max_from_cpuinfo(i, 1, 0);
		adjust_scaling_min_from_base_freq(i);
	}
}

static void set_scaling_min_to_cpuinfo_min(struct isst_id *id)
{
	int i;

	if (id->cpu < 0)
		return;

	for (i = 0; i < get_topo_max_cpus(); ++i) {
		if (!is_cpu_in_power_domain(i, id))
			continue;

		if (is_cpu_online(i) != 1)
			continue;

		adjust_scaling_max_from_base_freq(i);
		set_cpufreq_scaling_min_max_from_cpuinfo(i, 0, 0);
	}
}

static void set_scaling_max_to_cpuinfo_max(struct isst_id *id)
{
	int i;

	for (i = 0; i < get_topo_max_cpus(); ++i) {
		if (!is_cpu_in_power_domain(i, id))
			continue;

		set_cpufreq_scaling_min_max_from_cpuinfo(i, 1, 1);
	}
}

static int set_core_priority_and_min(struct isst_id *id, int mask_size,
				     cpu_set_t *cpu_mask, int min_high,
				     int min_low)
{
	int ret, i;

	if (!CPU_COUNT_S(mask_size, cpu_mask))
		return -1;

	ret = set_clos_param(id, 0, 0, 0, min_high, 0xff);
	if (ret)
		return ret;

	ret = set_clos_param(id, 1, 15, 15, min_low, 0xff);
	if (ret)
		return ret;

	ret = set_clos_param(id, 2, 15, 15, min_low, 0xff);
	if (ret)
		return ret;

	ret = set_clos_param(id, 3, 15, 15, min_low, 0xff);
	if (ret)
		return ret;

	for (i = 0; i < get_topo_max_cpus(); ++i) {
		int clos;
		struct isst_id tid;

		if (!is_cpu_in_power_domain(i, id))
			continue;

		if (CPU_ISSET_S(i, mask_size, cpu_mask))
			clos = 0;
		else
			clos = 3;

		debug_printf("Associate cpu: %d clos: %d\n", i, clos);
		set_isst_id(&tid, i);
		ret = isst_clos_associate(&tid, clos);
		if (ret) {
			isst_display_error_info_message(1, "isst_clos_associate failed", 0, 0);
			return ret;
		}
	}

	return 0;
}

static int set_pbf_core_power(struct isst_id *id)
{
	struct isst_pbf_info pbf_info;
	struct isst_pkg_ctdp pkg_dev;
	int ret;

	if (id->cpu < 0)
		return 0;

	ret = isst_get_ctdp_levels(id, &pkg_dev);
	if (ret) {
		debug_printf("isst_get_ctdp_levels failed");
		return ret;
	}
	debug_printf("Current_level: %d\n", pkg_dev.current_level);

	ret = isst_get_pbf_info(id, pkg_dev.current_level, &pbf_info);
	if (ret) {
		debug_printf("isst_get_pbf_info failed");
		return ret;
	}
	debug_printf("p1_high: %d p1_low: %d\n", pbf_info.p1_high,
		     pbf_info.p1_low);

	ret = set_core_priority_and_min(id, pbf_info.core_cpumask_size,
					pbf_info.core_cpumask,
					pbf_info.p1_high, pbf_info.p1_low);
	if (ret) {
		debug_printf("set_core_priority_and_min failed");
		return ret;
	}

	ret = isst_pm_qos_config(id, 1, 1);
	if (ret) {
		debug_printf("isst_pm_qos_config failed");
		return ret;
	}

	return 0;
}

static void set_pbf_for_cpu(struct isst_id *id, void *arg1, void *arg2, void *arg3,
			    void *arg4)
{
	struct isst_pkg_ctdp_level_info ctdp_level;
	struct isst_pkg_ctdp pkg_dev;
	int ret;
	int status = *(int *)arg4;

	if (is_clx_n_platform()) {
		ret = 0;
		if (status) {
			set_clx_pbf_cpufreq_scaling_min_max(id);

		} else {
			set_scaling_max_to_cpuinfo_max(id);
			set_scaling_min_to_cpuinfo_min(id);
		}
		goto disp_result;
	}

	ret = isst_get_ctdp_levels(id, &pkg_dev);
	if (ret) {
		isst_display_error_info_message(1, "Failed to get number of levels", 0, 0);
		goto disp_result;
	}

	ret = isst_get_ctdp_control(id, pkg_dev.current_level, &ctdp_level);
	if (ret) {
		isst_display_error_info_message(1, "Failed to get current level", 0, 0);
		goto disp_result;
	}

	if (!ctdp_level.pbf_support) {
		isst_display_error_info_message(1, "base-freq feature is not present at this level", 1, pkg_dev.current_level);
		ret = -1;
		goto disp_result;
	}

	if (auto_mode && status) {
		ret = set_pbf_core_power(id);
		if (ret)
			goto disp_result;
	}

	ret = isst_set_pbf_fact_status(id, 1, status);
	if (ret) {
		debug_printf("isst_set_pbf_fact_status failed");
		if (auto_mode)
			isst_pm_qos_config(id, 0, 0);
	} else {
		if (auto_mode) {
			if (status)
				set_scaling_min_to_cpuinfo_max(id);
			else
				set_scaling_min_to_cpuinfo_min(id);
		}
	}

	if (auto_mode && !status)
		isst_pm_qos_config(id, 0, 1);

disp_result:
	if (status)
		isst_display_result(id, outf, "base-freq", "enable",
				    ret);
	else
		isst_display_result(id, outf, "base-freq", "disable",
				    ret);
}

static void set_pbf_enable(int arg)
{
	int enable = arg;

	if (cmd_help) {
		if (enable) {
			fprintf(stderr,
				"Enable Intel Speed Select Technology base frequency feature\n");
			if (is_clx_n_platform()) {
				fprintf(stderr,
					"\tOn this platform this command doesn't enable feature in the hardware.\n");
				fprintf(stderr,
					"\tIt updates the cpufreq scaling_min_freq to match cpufreq base_frequency.\n");
				exit(0);

			}
			fprintf(stderr,
				"\tOptional Arguments: -a|--auto : Use priority of cores to set core-power associations\n");
		} else {

			if (is_clx_n_platform()) {
				fprintf(stderr,
					"\tOn this platform this command doesn't disable feature in the hardware.\n");
				fprintf(stderr,
					"\tIt updates the cpufreq scaling_min_freq to match cpuinfo_min_freq\n");
				exit(0);
			}
			fprintf(stderr,
				"Disable Intel Speed Select Technology base frequency feature\n");
			fprintf(stderr,
				"\tOptional Arguments: -a|--auto : Also disable core-power associations\n");
		}
		exit(0);
	}

	isst_ctdp_display_information_start(outf);
	if (max_target_cpus)
		for_each_online_target_cpu_in_set(set_pbf_for_cpu, NULL, NULL,
						  NULL, &enable);
	else
		for_each_online_power_domain_in_set(set_pbf_for_cpu, NULL, NULL,
					       NULL, &enable);
	isst_ctdp_display_information_end(outf);
}

static void dump_fact_config_for_cpu(struct isst_id *id, void *arg1, void *arg2,
				     void *arg3, void *arg4)
{
	struct isst_fact_info fact_info;
	int ret;

	memset(&fact_info, 0, sizeof(fact_info));
	ret = isst_get_fact_info(id, tdp_level, fact_bucket, &fact_info);
	if (ret) {
		isst_display_error_info_message(1, "Failed to get turbo-freq info at this level", 1, tdp_level);
		isst_ctdp_display_information_end(outf);
		exit(1);
	} else {
		isst_fact_display_information(id, outf, tdp_level, fact_bucket,
					      fact_avx, &fact_info);
	}
}

static void dump_fact_config(int arg)
{
	if (cmd_help) {
		fprintf(stderr,
			"Print complete Intel Speed Select Technology turbo frequency configuration for a TDP level. Other arguments are optional.\n");
		fprintf(stderr,
			"\tArguments: -l|--level : Specify tdp level\n");
		fprintf(stderr,
			"\tArguments: -b|--bucket : Bucket index to dump\n");
		fprintf(stderr,
			"\tArguments: -r|--trl-type : Specify trl type: sse|avx2|avx512\n");
		exit(0);
	}

	if (tdp_level == 0xff) {
		isst_display_error_info_message(1, "Invalid command: specify tdp_level\n", 0, 0);
		exit(1);
	}

	isst_ctdp_display_information_start(outf);
	if (max_target_cpus)
		for_each_online_target_cpu_in_set(dump_fact_config_for_cpu,
						  NULL, NULL, NULL, NULL);
	else
		for_each_online_power_domain_in_set(dump_fact_config_for_cpu, NULL,
					       NULL, NULL, NULL);
	isst_ctdp_display_information_end(outf);
}

static void set_fact_for_cpu(struct isst_id *id, void *arg1, void *arg2, void *arg3,
			     void *arg4)
{
	struct isst_pkg_ctdp_level_info ctdp_level;
	struct isst_pkg_ctdp pkg_dev;
	int ret;
	int status = *(int *)arg4;

	if (status && no_turbo()) {
		isst_display_error_info_message(1, "Turbo mode is disabled", 0, 0);
		ret = -1;
		goto disp_results;
	}

	ret = isst_get_ctdp_levels(id, &pkg_dev);
	if (ret) {
		isst_display_error_info_message(1, "Failed to get number of levels", 0, 0);
		goto disp_results;
	}

	ret = isst_get_ctdp_control(id, pkg_dev.current_level, &ctdp_level);
	if (ret) {
		isst_display_error_info_message(1, "Failed to get current level", 0, 0);
		goto disp_results;
	}

	if (!ctdp_level.fact_support) {
		isst_display_error_info_message(1, "turbo-freq feature is not present at this level", 1, pkg_dev.current_level);
		ret = -1;
		goto disp_results;
	}

	if (status) {
		ret = isst_pm_qos_config(id, 1, 1);
		if (ret)
			goto disp_results;
	}

	ret = isst_set_pbf_fact_status(id, 0, status);
	if (ret) {
		debug_printf("isst_set_pbf_fact_status failed");
		if (auto_mode)
			isst_pm_qos_config(id, 0, 0);

		goto disp_results;
	}

	/* Set TRL */
	if (status) {
		struct isst_pkg_ctdp pkg_dev;

		ret = isst_get_ctdp_levels(id, &pkg_dev);
		if (!ret && id->cpu >= 0)
			ret = isst_set_trl(id, fact_trl);
		if (ret && auto_mode)
			isst_pm_qos_config(id, 0, 0);
	} else {
		if (auto_mode)
			isst_pm_qos_config(id, 0, 0);
	}

disp_results:
	if (status) {
		isst_display_result(id, outf, "turbo-freq", "enable", ret);
		if (ret)
			fact_enable_fail = ret;
	} else {
		/* Since we modified TRL during Fact enable, restore it */
		isst_set_trl_from_current_tdp(id, fact_trl);
		isst_display_result(id, outf, "turbo-freq", "disable", ret);
	}
}

static void set_fact_enable(int arg)
{
	int i, ret, enable = arg;
	struct isst_id id;

	if (cmd_help) {
		if (enable) {
			fprintf(stderr,
				"Enable Intel Speed Select Technology Turbo frequency feature\n");
			fprintf(stderr,
				"Optional: -t|--trl : Specify turbo ratio limit in hex starting with 0x\n");
			fprintf(stderr,
				"\tOptional Arguments: -a|--auto : Designate specified target CPUs with");
			fprintf(stderr,
				"-C|--cpu option as as high priority using core-power feature\n");
		} else {
			fprintf(stderr,
				"Disable Intel Speed Select Technology turbo frequency feature\n");
			fprintf(stderr,
				"Optional: -t|--trl : Specify turbo ratio limit in hex starting with 0x\n");
			fprintf(stderr,
				"\tOptional Arguments: -a|--auto : Also disable core-power associations\n");
		}
		exit(0);
	}

	isst_ctdp_display_information_start(outf);
	if (max_target_cpus)
		for_each_online_target_cpu_in_set(set_fact_for_cpu, NULL, NULL,
						  NULL, &enable);
	else
		for_each_online_power_domain_in_set(set_fact_for_cpu, NULL, NULL,
					       NULL, &enable);

	if (!fact_enable_fail && enable && auto_mode) {
		/*
		 * When we adjust CLOS param, we have to set for siblings also.
		 * So for the each user specified CPU, also add the sibling
		 * in the present_cpu_mask.
		 */
		for (i = 0; i < get_topo_max_cpus(); ++i) {
			char buffer[128], sibling_list[128], *cpu_str;
			int fd, len;

			if (!CPU_ISSET_S(i, target_cpumask_size, target_cpumask))
				continue;

			snprintf(buffer, sizeof(buffer),
				 "/sys/devices/system/cpu/cpu%d/topology/thread_siblings_list", i);

			fd = open(buffer, O_RDONLY);
			if (fd < 0)
				continue;

			len = read(fd, sibling_list, sizeof(sibling_list));
			close(fd);

			if (len < 0)
				continue;

			sibling_list[127] = '\0';
			cpu_str = strtok(sibling_list, ",");
			while (cpu_str != NULL) {
				int cpu;

				sscanf(cpu_str, "%d", &cpu);
				CPU_SET_S(cpu, target_cpumask_size, target_cpumask);
				cpu_str = strtok(NULL, ",");
			}
		}

		for (i = 0; i < get_topo_max_cpus(); ++i) {
			int clos;

			if (!CPU_ISSET_S(i, present_cpumask_size, present_cpumask))
				continue;

			if (is_cpu_online(i) != 1)
				continue;

			set_isst_id(&id, i);
			ret = set_clos_param(&id, 0, 0, 0, 0, 0xff);
			if (ret)
				goto error_disp;

			ret = set_clos_param(&id, 1, 15, 15, 0, 0xff);
			if (ret)
				goto error_disp;

			ret = set_clos_param(&id, 2, 15, 15, 0, 0xff);
			if (ret)
				goto error_disp;

			ret = set_clos_param(&id, 3, 15, 15, 0, 0xff);
			if (ret)
				goto error_disp;

			if (CPU_ISSET_S(i, target_cpumask_size, target_cpumask))
				clos = 0;
			else
				clos = 3;

			debug_printf("Associate cpu: %d clos: %d\n", i, clos);
			ret = isst_clos_associate(&id, clos);
			if (ret)
				goto error_disp;
		}
		set_isst_id(&id, -1);
		isst_display_result(&id, outf, "turbo-freq --auto", "enable", 0);
	}

	isst_ctdp_display_information_end(outf);

	return;

error_disp:
	isst_display_result(&id, outf, "turbo-freq --auto", "enable", ret);
	isst_ctdp_display_information_end(outf);

}

static void enable_clos_qos_config(struct isst_id *id, void *arg1, void *arg2, void *arg3,
				   void *arg4)
{
	int ret;
	int status = *(int *)arg4;
	int cp_state, cp_cap;

	if (!isst_read_pm_config(id, &cp_state, &cp_cap)) {
		if (!cp_cap) {
			isst_display_error_info_message(1, "core-power not supported", 0, 0);
			return;
		}
	}

	if (is_skx_based_platform())
		clos_priority_type = 1;

	ret = isst_pm_qos_config(id, status, clos_priority_type);
	if (ret)
		isst_display_error_info_message(1, "isst_pm_qos_config failed", 0, 0);

	if (status)
		isst_display_result(id, outf, "core-power", "enable",
				    ret);
	else
		isst_display_result(id, outf, "core-power", "disable",
				    ret);
}

static void set_clos_enable(int arg)
{
	int enable = arg;

	if (cmd_help) {
		if (enable) {
			fprintf(stderr,
				"Enable core-power for a package/die\n");
			if (!is_skx_based_platform()) {
				fprintf(stderr,
					"\tClos Enable: Specify priority type with [--priority|-p]\n");
				fprintf(stderr, "\t\t 0: Proportional, 1: Ordered\n");
			}
		} else {
			fprintf(stderr,
				"Disable core-power: [No command arguments are required]\n");
		}
		exit(0);
	}

	if (enable && cpufreq_sysfs_present()) {
		fprintf(stderr,
			"cpufreq subsystem and core-power enable will interfere with each other!\n");
	}

	isst_ctdp_display_information_start(outf);
	if (max_target_cpus)
		for_each_online_target_cpu_in_set(enable_clos_qos_config, NULL,
						  NULL, NULL, &enable);
	else
		for_each_online_power_domain_in_set(enable_clos_qos_config, NULL,
					       NULL, NULL, &enable);
	isst_ctdp_display_information_end(outf);
}

static void dump_clos_config_for_cpu(struct isst_id *id, void *arg1, void *arg2,
				     void *arg3, void *arg4)
{
	struct isst_clos_config clos_config;
	int ret;

	ret = isst_pm_get_clos(id, current_clos, &clos_config);
	if (ret)
		isst_display_error_info_message(1, "isst_pm_get_clos failed", 0, 0);
	else
		isst_clos_display_information(id, outf, current_clos,
					      &clos_config);
}

static void dump_clos_config(int arg)
{
	if (cmd_help) {
		fprintf(stderr,
			"Print Intel Speed Select Technology core power configuration\n");
		fprintf(stderr,
			"\tArguments: [-c | --clos]: Specify clos id\n");
		exit(0);
	}
	if (current_clos < 0 || current_clos > 3) {
		isst_display_error_info_message(1, "Invalid clos id\n", 0, 0);
		isst_ctdp_display_information_end(outf);
		exit(0);
	}

	isst_ctdp_display_information_start(outf);
	if (max_target_cpus)
		for_each_online_target_cpu_in_set(dump_clos_config_for_cpu,
						  NULL, NULL, NULL, NULL);
	else
		for_each_online_power_domain_in_set(dump_clos_config_for_cpu, NULL,
					       NULL, NULL, NULL);
	isst_ctdp_display_information_end(outf);
}

static void get_clos_info_for_cpu(struct isst_id *id, void *arg1, void *arg2, void *arg3,
				  void *arg4)
{
	int enable, ret, prio_type;

	ret = isst_clos_get_clos_information(id, &enable, &prio_type);
	if (ret)
		isst_display_error_info_message(1, "isst_clos_get_info failed", 0, 0);
	else {
		int cp_state, cp_cap;

		isst_read_pm_config(id, &cp_state, &cp_cap);
		isst_clos_display_clos_information(id, outf, enable, prio_type,
						   cp_state, cp_cap);
	}
}

static void dump_clos_info(int arg)
{
	if (cmd_help) {
		fprintf(stderr,
			"Print Intel Speed Select Technology core power information\n");
		fprintf(stderr, "\t Optionally specify targeted cpu id with [--cpu|-c]\n");
		exit(0);
	}

	isst_ctdp_display_information_start(outf);
	if (max_target_cpus)
		for_each_online_target_cpu_in_set(get_clos_info_for_cpu, NULL,
						  NULL, NULL, NULL);
	else
		for_each_online_power_domain_in_set(get_clos_info_for_cpu, NULL,
					       NULL, NULL, NULL);
	isst_ctdp_display_information_end(outf);

}

static void set_clos_config_for_cpu(struct isst_id *id, void *arg1, void *arg2, void *arg3,
				    void *arg4)
{
	struct isst_clos_config clos_config;
	int ret;

	if (id->cpu < 0)
		return;

	clos_config.epp = clos_epp;
	clos_config.clos_prop_prio = clos_prop_prio;
	clos_config.clos_min = clos_min;
	clos_config.clos_max = clos_max;
	clos_config.clos_desired = clos_desired;
	ret = isst_set_clos(id, current_clos, &clos_config);
	if (ret)
		isst_display_error_info_message(1, "isst_set_clos failed", 0, 0);
	else
		isst_display_result(id, outf, "core-power", "config", ret);
}

static void set_clos_config(int arg)
{
	if (cmd_help) {
		fprintf(stderr,
			"Set core-power configuration for one of the four clos ids\n");
		fprintf(stderr,
			"\tSpecify targeted clos id with [--clos|-c]\n");
		if (!is_skx_based_platform()) {
			fprintf(stderr, "\tSpecify clos EPP with [--epp|-e]\n");
			fprintf(stderr,
				"\tSpecify clos Proportional Priority [--weight|-w]\n");
		}
		fprintf(stderr, "\tSpecify clos min in MHz with [--min|-n]\n");
		fprintf(stderr, "\tSpecify clos max in MHz with [--max|-m]\n");
		exit(0);
	}

	if (current_clos < 0 || current_clos > 3) {
		isst_display_error_info_message(1, "Invalid clos id\n", 0, 0);
		exit(0);
	}
	if (!is_skx_based_platform() && (clos_epp < 0 || clos_epp > 0x0F)) {
		fprintf(stderr, "clos epp is not specified or invalid, default: 0\n");
		clos_epp = 0;
	}
	if (!is_skx_based_platform() && (clos_prop_prio < 0 || clos_prop_prio > 0x0F)) {
		fprintf(stderr,
			"clos frequency weight is not specified or invalid, default: 0\n");
		clos_prop_prio = 0;
	}
	if (clos_min < 0) {
		fprintf(stderr, "clos min is not specified, default: 0\n");
		clos_min = 0;
	}
	if (clos_max < 0) {
		fprintf(stderr, "clos max is not specified, default: Max frequency (ratio 0xff)\n");
		clos_max = 0xff;
	}
	if (clos_desired) {
		fprintf(stderr, "clos desired is not supported on this platform\n");
		clos_desired = 0x00;
	}

	isst_ctdp_display_information_start(outf);
	if (max_target_cpus)
		for_each_online_target_cpu_in_set(set_clos_config_for_cpu, NULL,
						  NULL, NULL, NULL);
	else
		for_each_online_power_domain_in_set(set_clos_config_for_cpu, NULL,
					       NULL, NULL, NULL);
	isst_ctdp_display_information_end(outf);
}

static void set_clos_assoc_for_cpu(struct isst_id *id, void *arg1, void *arg2, void *arg3,
				   void *arg4)
{
	int ret;

	ret = isst_clos_associate(id, current_clos);
	if (ret)
		debug_printf("isst_clos_associate failed");
	else
		isst_display_result(id, outf, "core-power", "assoc", ret);
}

static void set_clos_assoc(int arg)
{
	if (cmd_help) {
		fprintf(stderr, "Associate a clos id to a CPU\n");
		fprintf(stderr,
			"\tSpecify targeted clos id with [--clos|-c]\n");
		fprintf(stderr,
			"\tFor example to associate clos 1 to CPU 0: issue\n");
		fprintf(stderr,
			"\tintel-speed-select --cpu 0 core-power assoc --clos 1\n");
		exit(0);
	}

	if (current_clos < 0 || current_clos > 3) {
		isst_display_error_info_message(1, "Invalid clos id\n", 0, 0);
		exit(0);
	}

	isst_ctdp_display_information_start(outf);

	if (max_target_cpus)
		for_each_online_target_cpu_in_set(set_clos_assoc_for_cpu, NULL,
						  NULL, NULL, NULL);
	else {
		isst_display_error_info_message(1, "Invalid target cpu. Specify with [-c|--cpu]", 0, 0);
	}
	isst_ctdp_display_information_end(outf);
}

static void get_clos_assoc_for_cpu(struct isst_id *id, void *arg1, void *arg2, void *arg3,
				   void *arg4)
{
	int clos, ret;

	ret = isst_clos_get_assoc_status(id, &clos);
	if (ret)
		isst_display_error_info_message(1, "isst_clos_get_assoc_status failed", 0, 0);
	else
		isst_clos_display_assoc_information(id, outf, clos);
}

static void get_clos_assoc(int arg)
{
	if (cmd_help) {
		fprintf(stderr, "Get associate clos id to a CPU\n");
		fprintf(stderr, "\tSpecify targeted cpu id with [--cpu|-c]\n");
		exit(0);
	}

	if (!max_target_cpus) {
		isst_display_error_info_message(1, "Invalid target cpu. Specify with [-c|--cpu]", 0, 0);
		exit(0);
	}

	isst_ctdp_display_information_start(outf);
	for_each_online_target_cpu_in_set(get_clos_assoc_for_cpu, NULL,
					  NULL, NULL, NULL);
	isst_ctdp_display_information_end(outf);
}

static void set_turbo_mode_for_cpu(struct isst_id *id, int status)
{
	int base_freq;

	if (status) {
		base_freq = get_cpufreq_base_freq(id->cpu);
		set_cpufreq_scaling_min_max(id->cpu, 1, base_freq);
	} else {
		set_scaling_max_to_cpuinfo_max(id);
	}

	if (status) {
		isst_display_result(id, outf, "turbo-mode", "disable", 0);
	} else {
		isst_display_result(id, outf, "turbo-mode", "enable", 0);
	}
}

static void set_turbo_mode(int arg)
{
	int i, disable = arg;
	struct isst_id id;

	if (cmd_help) {
		if (disable)
			fprintf(stderr, "Set turbo mode disable\n");
		else
			fprintf(stderr, "Set turbo mode enable\n");
		exit(0);
	}

	isst_ctdp_display_information_start(outf);

	for (i = 0; i < topo_max_cpus; ++i) {
		int online;

		if (i)
			online = parse_int_file(
				1, "/sys/devices/system/cpu/cpu%d/online", i);
		else
			online =
				1; /* online entry for CPU 0 needs some special configs */

		if (online) {
			set_isst_id(&id, i);
			set_turbo_mode_for_cpu(&id, disable);
		}

	}
	isst_ctdp_display_information_end(outf);
}

static void get_set_trl(struct isst_id *id, void *arg1, void *arg2, void *arg3,
			void *arg4)
{
	unsigned long long trl;
	int set = *(int *)arg4;
	int ret;

	if (id->cpu < 0)
		return;

	if (set && !fact_trl) {
		isst_display_error_info_message(1, "Invalid TRL. Specify with [-t|--trl]", 0, 0);
		exit(0);
	}

	if (set) {
		ret = isst_set_trl(id, fact_trl);
		isst_display_result(id, outf, "turbo-mode", "set-trl", ret);
		return;
	}

	ret = isst_get_trl(id, &trl);
	if (ret)
		isst_display_result(id, outf, "turbo-mode", "get-trl", ret);
	else
		isst_trl_display_information(id, outf, trl);
}

static void process_trl(int arg)
{
	if (cmd_help) {
		if (arg) {
			fprintf(stderr, "Set TRL (turbo ratio limits)\n");
			fprintf(stderr, "\t t|--trl: Specify turbo ratio limit for setting TRL in hex starting with 0x\n");
		} else {
			fprintf(stderr, "Get TRL (turbo ratio limits)\n");
		}
		exit(0);
	}

	isst_ctdp_display_information_start(outf);
	if (max_target_cpus)
		for_each_online_target_cpu_in_set(get_set_trl, NULL,
						  NULL, NULL, &arg);
	else
		for_each_online_power_domain_in_set(get_set_trl, NULL,
					       NULL, NULL, &arg);
	isst_ctdp_display_information_end(outf);
}

static struct process_cmd_struct clx_n_cmds[] = {
	{ "perf-profile", "info", dump_isst_config, 0 },
	{ "base-freq", "info", dump_pbf_config, 0 },
	{ "base-freq", "enable", set_pbf_enable, 1 },
	{ "base-freq", "disable", set_pbf_enable, 0 },
	{ NULL, NULL, NULL, 0 }
};

static struct process_cmd_struct isst_cmds[] = {
	{ "perf-profile", "get-lock-status", get_tdp_locked, 0 },
	{ "perf-profile", "get-config-levels", get_tdp_levels, 0 },
	{ "perf-profile", "get-config-version", get_tdp_version, 0 },
	{ "perf-profile", "get-config-enabled", get_tdp_enabled, 0 },
	{ "perf-profile", "get-config-current-level", get_tdp_current_level,
	 0 },
	{ "perf-profile", "set-config-level", set_tdp_level, 0 },
	{ "perf-profile", "info", dump_isst_config, 0 },
	{ "base-freq", "info", dump_pbf_config, 0 },
	{ "base-freq", "enable", set_pbf_enable, 1 },
	{ "base-freq", "disable", set_pbf_enable, 0 },
	{ "turbo-freq", "info", dump_fact_config, 0 },
	{ "turbo-freq", "enable", set_fact_enable, 1 },
	{ "turbo-freq", "disable", set_fact_enable, 0 },
	{ "core-power", "info", dump_clos_info, 0 },
	{ "core-power", "enable", set_clos_enable, 1 },
	{ "core-power", "disable", set_clos_enable, 0 },
	{ "core-power", "config", set_clos_config, 0 },
	{ "core-power", "get-config", dump_clos_config, 0 },
	{ "core-power", "assoc", set_clos_assoc, 0 },
	{ "core-power", "get-assoc", get_clos_assoc, 0 },
	{ "turbo-mode", "enable", set_turbo_mode, 0 },
	{ "turbo-mode", "disable", set_turbo_mode, 1 },
	{ "turbo-mode", "get-trl", process_trl, 0 },
	{ "turbo-mode", "set-trl", process_trl, 1 },
	{ NULL, NULL, NULL }
};

/*
 * parse cpuset with following syntax
 * 1,2,4..6,8-10 and set bits in cpu_subset
 */
void parse_cpu_command(char *optarg)
{
	unsigned int start, end, invalid_count;
	char *next;

	next = optarg;
	invalid_count = 0;

	while (next && *next) {
		if (*next == '-') /* no negative cpu numbers */
			goto error;

		start = strtoul(next, &next, 10);

		if (max_target_cpus < MAX_CPUS_IN_ONE_REQ)
			target_cpus[max_target_cpus++] = start;
		else
			invalid_count = 1;

		if (*next == '\0')
			break;

		if (*next == ',') {
			next += 1;
			continue;
		}

		if (*next == '-') {
			next += 1; /* start range */
		} else if (*next == '.') {
			next += 1;
			if (*next == '.')
				next += 1; /* start range */
			else
				goto error;
		}

		end = strtoul(next, &next, 10);
		if (end <= start)
			goto error;

		while (++start <= end) {
			if (max_target_cpus < MAX_CPUS_IN_ONE_REQ)
				target_cpus[max_target_cpus++] = start;
			else
				invalid_count = 1;
		}

		if (*next == ',')
			next += 1;
		else if (*next != '\0')
			goto error;
	}

	if (invalid_count) {
		isst_ctdp_display_information_start(outf);
		isst_display_error_info_message(1, "Too many CPUs in one request: max is", 1, MAX_CPUS_IN_ONE_REQ - 1);
		isst_ctdp_display_information_end(outf);
		exit(-1);
	}

#ifdef DEBUG
	{
		int i;

		for (i = 0; i < max_target_cpus; ++i)
			printf("cpu [%d] in arg\n", target_cpus[i]);
	}
#endif
	return;

error:
	fprintf(stderr, "\"--cpu %s\" malformed\n", optarg);
	exit(-1);
}

static void check_optarg(char *option, int hex)
{
	if (optarg) {
		char *start = optarg;
		int i;

		if (hex && strlen(optarg) < 3) {
			/* At least 0x plus one character must be present */
			fprintf(stderr, "malformed arguments for:%s [%s]\n", option, optarg);
			exit(0);
		}

		if (hex) {
			if (optarg[0] != '0' || tolower(optarg[1]) != 'x') {
				fprintf(stderr, "malformed arguments for:%s [%s]\n",
					option, optarg);
				exit(0);
			}
			start = &optarg[2];
		}

		for (i = 0; i < strlen(start); ++i) {
			if (hex) {
				if (!isxdigit(start[i])) {
					fprintf(stderr, "malformed arguments for:%s [%s]\n",
						option, optarg);
					exit(0);
				}
			} else if (!isdigit(start[i])) {
				fprintf(stderr, "malformed arguments for:%s [%s]\n",
					option, optarg);
				exit(0);
			}
		}
	}
}

static void parse_cmd_args(int argc, int start, char **argv)
{
	int opt;
	int option_index;

	static struct option long_options[] = {
		{ "bucket", required_argument, 0, 'b' },
		{ "level", required_argument, 0, 'l' },
		{ "online", required_argument, 0, 'o' },
		{ "trl-type", required_argument, 0, 'r' },
		{ "trl", required_argument, 0, 't' },
		{ "help", no_argument, 0, 'h' },
		{ "clos", required_argument, 0, 'c' },
		{ "desired", required_argument, 0, 'd' },
		{ "epp", required_argument, 0, 'e' },
		{ "min", required_argument, 0, 'n' },
		{ "max", required_argument, 0, 'm' },
		{ "priority", required_argument, 0, 'p' },
		{ "weight", required_argument, 0, 'w' },
		{ "auto", no_argument, 0, 'a' },
		{ 0, 0, 0, 0 }
	};

	option_index = start;

	optind = start + 1;
	while ((opt = getopt_long(argc, argv, "b:l:t:c:d:e:n:m:p:w:r:hoa",
				  long_options, &option_index)) != -1) {
		switch (opt) {
		case 'a':
			auto_mode = 1;
			break;
		case 'b':
			check_optarg("bucket", 0);
			fact_bucket = atoi(optarg);
			break;
		case 'h':
			cmd_help = 1;
			break;
		case 'l':
			check_optarg("level", 0);
			tdp_level = atoi(optarg);
			break;
		case 'o':
			force_online_offline = 1;
			break;
		case 't':
			check_optarg("trl", 1);
			sscanf(optarg, "0x%llx", &fact_trl);
			break;
		case 'r':
			if (!strncmp(optarg, "sse", 3)) {
				fact_avx = 0x01;
			} else if (!strncmp(optarg, "avx2", 4)) {
				fact_avx = 0x02;
			} else if (!strncmp(optarg, "avx512", 6)) {
				fact_avx = 0x04;
			} else {
				fprintf(outf, "Invalid sse,avx options\n");
				exit(1);
			}
			break;
		/* CLOS related */
		case 'c':
			check_optarg("clos", 0);
			current_clos = atoi(optarg);
			break;
		case 'd':
			check_optarg("desired", 0);
			clos_desired = atoi(optarg);
			clos_desired /= isst_get_disp_freq_multiplier();
			break;
		case 'e':
			check_optarg("epp", 0);
			clos_epp = atoi(optarg);
			if (is_skx_based_platform()) {
				isst_display_error_info_message(1, "epp can't be specified on this platform", 0, 0);
				exit(0);
			}
			break;
		case 'n':
			check_optarg("min", 0);
			clos_min = atoi(optarg);
			clos_min /= isst_get_disp_freq_multiplier();
			break;
		case 'm':
			check_optarg("max", 0);
			clos_max = atoi(optarg);
			clos_max /= isst_get_disp_freq_multiplier();
			break;
		case 'p':
			check_optarg("priority", 0);
			clos_priority_type = atoi(optarg);
			if (is_skx_based_platform() && !clos_priority_type) {
				isst_display_error_info_message(1, "Invalid clos priority type: proportional for this platform", 0, 0);
				exit(0);
			}
			break;
		case 'w':
			check_optarg("weight", 0);
			clos_prop_prio = atoi(optarg);
			if (is_skx_based_platform()) {
				isst_display_error_info_message(1, "weight can't be specified on this platform", 0, 0);
				exit(0);
			}
			break;
		default:
			printf("Unknown option: ignore\n");
		}
	}

	if (argv[optind])
		printf("Garbage at the end of command: ignore\n");
}

static void isst_help(void)
{
	printf("perf-profile:\tAn architectural mechanism that allows multiple optimized \n\
		performance profiles per system via static and/or dynamic\n\
		adjustment of core count, workload, Tjmax, and\n\
		TDP, etc.\n");
	printf("\nCommands : For feature=perf-profile\n");
	printf("\tinfo\n");

	if (!is_clx_n_platform()) {
		printf("\tget-lock-status\n");
		printf("\tget-config-levels\n");
		printf("\tget-config-version\n");
		printf("\tget-config-enabled\n");
		printf("\tget-config-current-level\n");
		printf("\tset-config-level\n");
	}
}

static void pbf_help(void)
{
	printf("base-freq:\tEnables users to increase guaranteed base frequency\n\
		on certain cores (high priority cores) in exchange for lower\n\
		base frequency on remaining cores (low priority cores).\n");
	printf("\tcommand : info\n");
	printf("\tcommand : enable\n");
	printf("\tcommand : disable\n");
}

static void fact_help(void)
{
	printf("turbo-freq:\tEnables the ability to set different turbo ratio\n\
		limits to cores based on priority.\n");
	printf("\nCommand: For feature=turbo-freq\n");
	printf("\tcommand : info\n");
	printf("\tcommand : enable\n");
	printf("\tcommand : disable\n");
}

static void turbo_mode_help(void)
{
	printf("turbo-mode:\tEnables users to enable/disable turbo mode by adjusting frequency settings. Also allows to get and set turbo ratio limits (TRL).\n");
	printf("\tcommand : enable\n");
	printf("\tcommand : disable\n");
	printf("\tcommand : get-trl\n");
	printf("\tcommand : set-trl\n");
}


static void core_power_help(void)
{
	printf("core-power:\tInterface that allows user to define per core/tile\n\
		priority.\n");
	printf("\nCommands : For feature=core-power\n");
	printf("\tinfo\n");
	printf("\tenable\n");
	printf("\tdisable\n");
	printf("\tconfig\n");
	printf("\tget-config\n");
	printf("\tassoc\n");
	printf("\tget-assoc\n");
}

struct process_cmd_help_struct {
	char *feature;
	void (*process_fn)(void);
};

static struct process_cmd_help_struct isst_help_cmds[] = {
	{ "perf-profile", isst_help },
	{ "base-freq", pbf_help },
	{ "turbo-freq", fact_help },
	{ "core-power", core_power_help },
	{ "turbo-mode", turbo_mode_help },
	{ NULL, NULL }
};

static struct process_cmd_help_struct clx_n_help_cmds[] = {
	{ "perf-profile", isst_help },
	{ "base-freq", pbf_help },
	{ NULL, NULL }
};

void process_command(int argc, char **argv,
		     struct process_cmd_help_struct *help_cmds,
		     struct process_cmd_struct *cmds)
{
	int i = 0, matched = 0;
	char *feature = argv[optind];
	char *cmd = argv[optind + 1];

	if (!feature || !cmd)
		return;

	debug_printf("feature name [%s] command [%s]\n", feature, cmd);
	if (!strcmp(cmd, "-h") || !strcmp(cmd, "--help")) {
		while (help_cmds[i].feature) {
			if (!strcmp(help_cmds[i].feature, feature)) {
				help_cmds[i].process_fn();
				exit(0);
			}
			++i;
		}
	}

	i = 0;
	while (cmds[i].feature) {
		if (!strcmp(cmds[i].feature, feature) &&
		    !strcmp(cmds[i].command, cmd)) {
			parse_cmd_args(argc, optind + 1, argv);
			cmds[i].process_fn(cmds[i].arg);
			matched = 1;
			break;
		}
		++i;
	}

	if (!matched)
		fprintf(stderr, "Invalid command\n");
}

static void usage(void)
{
	if (is_clx_n_platform()) {
		fprintf(stderr, "\nThere is limited support of Intel Speed Select features on this platform.\n");
		fprintf(stderr, "Everything is pre-configured using BIOS options, this tool can't enable any feature in the hardware.\n\n");
	}

	printf("\nUsage:\n");
	printf("intel-speed-select [OPTIONS] FEATURE COMMAND COMMAND_ARGUMENTS\n");
	printf("\nUse this tool to enumerate and control the Intel Speed Select Technology features:\n");
	if (is_clx_n_platform())
		printf("\nFEATURE : [perf-profile|base-freq]\n");
	else
		printf("\nFEATURE : [perf-profile|base-freq|turbo-freq|core-power|turbo-mode]\n");
	printf("\nFor help on each feature, use -h|--help\n");
	printf("\tFor example:  intel-speed-select perf-profile -h\n");

	printf("\nFor additional help on each command for a feature, use --h|--help\n");
	printf("\tFor example:  intel-speed-select perf-profile get-lock-status -h\n");
	printf("\t\t This will print help for the command \"get-lock-status\" for the feature \"perf-profile\"\n");

	printf("\nOPTIONS\n");
	printf("\t[-c|--cpu] : logical cpu number\n");
	printf("\t\tDefault: Die scoped for all dies in the system with multiple dies/package\n");
	printf("\t\t\t Or Package scoped for all Packages when each package contains one die\n");
	printf("\t[-d|--debug] : Debug mode\n");
	printf("\t[-f|--format] : output format [json|text]. Default: text\n");
	printf("\t[-h|--help] : Print help\n");
	printf("\t[-i|--info] : Print platform information\n");
	printf("\t[-a|--all-cpus-online] : Force online every CPU in the system\n");
	printf("\t[-o|--out] : Output file\n");
	printf("\t\t\tDefault : stderr\n");
	printf("\t[-p|--pause] : Delay between two mail box commands in milliseconds\n");
	printf("\t[-r|--retry] : Retry count for mail box commands on failure, default 3\n");
	printf("\t[-v|--version] : Print version\n");
	printf("\t[-b|--oob : Start a daemon to process HFI events for perf profile change from Out of Band agent.\n");
	printf("\t[-n|--no-daemon : Don't run as daemon. By default --oob will turn on daemon mode\n");
	printf("\t[-w|--delay : Delay for reading config level state change in OOB poll mode.\n");
	printf("\t[-g|--cgroupv2 : Try to use cgroup v2 CPU isolation instead of CPU online/offline.\n");
	printf("\t[-u|--cpu0-workaround : Don't try to online/offline CPU0 instead use cgroup v2.\n");
	printf("\nResult format\n");
	printf("\tResult display uses a common format for each command:\n");
	printf("\tResults are formatted in text/JSON with\n");
	printf("\t\tPackage, Die, CPU, and command specific results.\n");

	printf("\nExamples\n");
	printf("\tTo get platform information:\n");
	printf("\t\tintel-speed-select --info\n");
	printf("\tTo get full perf-profile information dump:\n");
	printf("\t\tintel-speed-select perf-profile info\n");
	printf("\tTo get full base-freq information dump:\n");
	printf("\t\tintel-speed-select base-freq info -l 0\n");
	if (!is_clx_n_platform()) {
		printf("\tTo get full turbo-freq information dump:\n");
		printf("\t\tintel-speed-select turbo-freq info -l 0\n");
	}
	exit(1);
}

static void print_version(void)
{
	fprintf(outf, "Version %s\n", version_str);
	exit(0);
}

static void cmdline(int argc, char **argv)
{
	const char *pathname = "/dev/isst_interface";
	char *ptr;
	FILE *fp;
	int opt, force_cpus_online = 0;
	int option_index = 0;
	int ret;
	int oob_mode = 0;
	int poll_interval = -1;
	int no_daemon = 0;
	int mbox_delay = 0, mbox_retries = 3;

	static struct option long_options[] = {
		{ "all-cpus-online", no_argument, 0, 'a' },
		{ "cpu", required_argument, 0, 'c' },
		{ "debug", no_argument, 0, 'd' },
		{ "format", required_argument, 0, 'f' },
		{ "help", no_argument, 0, 'h' },
		{ "info", no_argument, 0, 'i' },
		{ "pause", required_argument, 0, 'p' },
		{ "out", required_argument, 0, 'o' },
		{ "retry", required_argument, 0, 'r' },
		{ "version", no_argument, 0, 'v' },
		{ "oob", no_argument, 0, 'b' },
		{ "no-daemon", no_argument, 0, 'n' },
		{ "poll-interval", required_argument, 0, 'w' },
		{ "cgroupv2", required_argument, 0, 'g' },
		{ "cpu0-workaround", required_argument, 0, 'u' },
		{ 0, 0, 0, 0 }
	};

	if (geteuid() != 0) {
		fprintf(stderr, "Must run as root\n");
		exit(0);
	}

	ret = update_cpu_model();
	if (ret)
		err(-1, "Invalid CPU model (%d)\n", cpu_model);
	printf("Intel(R) Speed Select Technology\n");
	printf("Executing on CPU model:%d[0x%x]\n", cpu_model, cpu_model);

	if (!is_clx_n_platform()) {
		fp = fopen(pathname, "rb");
		if (!fp) {
			fprintf(stderr, "Intel speed select drivers are not loaded on this system.\n");
			fprintf(stderr, "Verify that kernel config includes CONFIG_INTEL_SPEED_SELECT_INTERFACE.\n");
			fprintf(stderr, "If the config is included then this is not a supported platform.\n");
			exit(0);
		}
		fclose(fp);
	}

	ret = isst_fill_platform_info();
	if (ret)
		goto out;

	progname = argv[0];
	while ((opt = getopt_long_only(argc, argv, "+c:df:hio:vabw:ngu", long_options,
				       &option_index)) != -1) {
		switch (opt) {
		case 'a':
			force_cpus_online = 1;
			break;
		case 'c':
			parse_cpu_command(optarg);
			break;
		case 'd':
			debug_flag = 1;
			printf("Debug Mode ON\n");
			break;
		case 'f':
			if (!strncmp(optarg, "json", 4))
				out_format_json = 1;
			break;
		case 'h':
			usage();
			break;
		case 'i':
			isst_print_platform_information();
			break;
		case 'o':
			if (outf)
				fclose(outf);
			outf = fopen_or_exit(optarg, "w");
			break;
		case 'p':
			ret = strtol(optarg, &ptr, 10);
			if (!ret)
				fprintf(stderr, "Invalid pause interval, ignore\n");
			else
				mbox_delay = ret;
			break;
		case 'r':
			ret = strtol(optarg, &ptr, 10);
			if (!ret)
				fprintf(stderr, "Invalid retry count, ignore\n");
			else
				mbox_retries = ret;
			break;
		case 'v':
			print_version();
			break;
		case 'b':
			oob_mode = 1;
			break;
		case 'n':
			no_daemon = 1;
			break;
		case 'w':
			ret = strtol(optarg, &ptr, 10);
			if (!ret) {
				fprintf(stderr, "Invalid poll interval count\n");
				exit(0);
			}
			poll_interval = ret;
			break;
		case 'g':
			cgroupv2 = 1;
			break;
		case 'u':
			cpu_0_cgroupv2 = 1;
			break;
		default:
			usage();
		}
	}

	if (optind > (argc - 2) && !oob_mode) {
		usage();
		exit(0);
	}

	isst_update_platform_param(ISST_PARAM_MBOX_DELAY, mbox_delay);
	isst_update_platform_param(ISST_PARAM_MBOX_RETRIES, mbox_retries);

	set_max_cpu_num();
	if (force_cpus_online)
		force_all_cpus_online();
	store_cpu_topology();
	create_cpu_map();

	if (oob_mode) {
		if (debug_flag)
			fprintf(stderr, "OOB mode is enabled in debug mode\n");

		ret = isst_daemon(debug_flag, poll_interval, no_daemon);
		if (ret)
			fprintf(stderr, "OOB mode enable failed\n");
		goto out;
	}

	if (!is_clx_n_platform()) {
		process_command(argc, argv, isst_help_cmds, isst_cmds);
	} else {
		process_command(argc, argv, clx_n_help_cmds, clx_n_cmds);
	}
out:
	free_cpu_set(present_cpumask);
	free_cpu_set(target_cpumask);
}

int main(int argc, char **argv)
{
	outf = stderr;
	cmdline(argc, argv);
	return 0;
}
