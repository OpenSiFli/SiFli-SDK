# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Exercise real memheap.c with an acquire-refresh semaphore dependency.

This host unit test models the heap-lock/cache-acquire seam. It proves that
allocation does not decide capacity before acquiring the lock, and checks lock
release on real allocation failures. It does not model physical cache behavior.
Activate the SDK environment; set SMP_TEST_CC to a native GCC if not on PATH.
"""

import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "rtos" / "rtthread" / "src" / "memheap.c"
HEADERS = ROOT / "rtos" / "rtthread" / "include"
CONFIG = """\
#include <stdint.h>
#include <stddef.h>
/* The host CRT owns POSIX types; allocator types still come from rtdef.h. */
#define RTLIBC_H__
#define RT_USING_HEAP
#define RT_USING_MEMHEAP
#define RT_USING_SEMAPHORE
#define RT_NAME_MAX 8
#define RT_ALIGN_SIZE 8
#define RT_THREAD_PRIORITY_MAX 32
#define RT_TICK_PER_SECOND 1000
"""

HARNESS = r"""
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <rtthread.h>

/* memheap.c uses 32-bit pool-address arithmetic. Map real writable storage
 * below 4 GiB; do not replace the allocator or rewrite its pointer arithmetic. */
#ifdef _WIN32
__declspec(dllimport) void *__stdcall VirtualAlloc(void *, size_t, unsigned long, unsigned long);
__declspec(dllimport) int __stdcall VirtualFree(void *, size_t, unsigned long);
static void *pool_map(void)
{
    return VirtualAlloc((void *)(uintptr_t)0x21000000u, 65536, 0x3000, 0x04);
}
static void pool_unmap(void *pool) { VirtualFree(pool, 0, 0x8000); }
#else
#include <sys/mman.h>
static void *pool_map(void)
{
    void *pool = mmap((void *)(uintptr_t)0x21000000u, 65536,
                      PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    return pool == MAP_FAILED ? NULL : pool;
}
static void pool_unmap(void *pool) { munmap(pool, 65536); }
#endif

#define POOL_BYTES 4096
static struct rt_memheap heap;
static unsigned takes, releases, held, lock_errors;
static unsigned refresh_on_acquire;
static rt_uint32_t fresh_capacity;
static rt_err_t saved_errno;

/* Only dependencies are substituted. The DUT is the unmodified allocator. */
void rt_object_init(struct rt_object *object, enum rt_object_class_type type, const char *name)
{
    object->type = type | RT_Object_Class_Static;
    strncpy(object->name, name, RT_NAME_MAX);
    rt_list_init(&object->list);
}
void rt_object_detach(rt_object_t object) { (void)object; }
rt_uint8_t rt_object_get_type(rt_object_t object) { return object->type & ~RT_Object_Class_Static; }
rt_bool_t rt_object_is_systemobject(rt_object_t object) { return !!(object->type & RT_Object_Class_Static); }
rt_err_t rt_sem_init(rt_sem_t sem, const char *name, rt_uint32_t value, rt_uint8_t flag)
{
    (void)name;
    (void)flag;
    sem->value = value;
    return RT_EOK;
}
rt_err_t rt_sem_take(rt_sem_t sem, rt_int32_t timeout)
{
    (void)timeout;
    takes++;
    if (sem != &heap.lock || held) lock_errors++;
    held = 1;
    if (refresh_on_acquire)
    {
        heap.available_size = fresh_capacity;
        refresh_on_acquire = 0;
    }
    return RT_EOK;
}
rt_err_t rt_sem_release(rt_sem_t sem)
{
    releases++;
    if (sem != &heap.lock || !held) lock_errors++;
    held = 0;
    return RT_EOK;
}
void rt_set_errno(rt_err_t error) { saved_errno = error; }
void *rt_memcpy(void *dst, const void *src, rt_ubase_t count) { return memcpy(dst, src, count); }
void *rt_memset(void *dst, int value, rt_ubase_t count) { return memset(dst, value, count); }

#define CHECK(condition, reason) do { \
    if (!(condition)) { \
        printf("FAIL %s: %s takes=%u releases=%u held=%u errors=%u\n", \
               scenario, reason, takes, releases, held, lock_errors); \
        return 1; \
    } \
} while (0)

static void reset_counts(void) { takes = releases = lock_errors = 0; }

static int exercise(const char *scenario, void *pool)
{
    void *allocated;
    CHECK(rt_memheap_init(&heap, "fixture", pool, POOL_BYTES) == RT_EOK, "init failed");
    fresh_capacity = heap.available_size;
    CHECK(fresh_capacity > 256, "fixture has no free capacity");

    if (strcmp(scenario, "stale") == 0)
    {
        /* The actual free list has space; only the pre-acquire cached capacity
         * is stale. The semaphore dependency publishes its refreshed value. */
        heap.available_size = 0;
        refresh_on_acquire = 1;
        allocated = rt_memheap_alloc(&heap, 64);
        CHECK(allocated != NULL, "stale capacity rejected before acquire");
        CHECK(takes == 1 && releases == 1 && !held && !lock_errors,
              "successful allocation did not balance the heap lock");
        CHECK((uintptr_t)allocated >= (uintptr_t)pool &&
              (uintptr_t)allocated + 64 <= (uintptr_t)pool + POOL_BYTES,
              "allocation escaped the real pool");
        memset(allocated, 0x5a, 64);
        rt_memheap_free(allocated);
        CHECK(heap.available_size == fresh_capacity, "free did not restore capacity");
        CHECK(takes == 2 && releases == 2 && !held && !lock_errors,
              "free did not balance the heap lock");
    }
    else if (strcmp(scenario, "oom") == 0)
    {
        allocated = rt_memheap_alloc(&heap, POOL_BYTES * 2);
        CHECK(allocated == NULL, "oversized allocation succeeded");
        CHECK(takes == 1 && releases == 1 && !held && !lock_errors,
              "capacity failure did not release the acquired lock");
        CHECK(heap.available_size == fresh_capacity, "capacity failure changed the heap");
        allocated = rt_memheap_alloc(&heap, 64);
        CHECK(allocated != NULL, "allocation after OOM failed");
        rt_memheap_free(allocated);
        CHECK(heap.available_size == fresh_capacity && !held && !lock_errors,
              "OOM damaged subsequent allocation/free");
    }
    else if (strcmp(scenario, "fragmented") == 0)
    {
        void *blocks[64];
        unsigned count = 0;
        while (count < 64 && (blocks[count] = rt_memheap_alloc(&heap, 128)) != NULL) count++;
        CHECK(count >= 4 && count < 64, "fixture did not fill the heap");
        for (unsigned i = 0; i < count; i += 2) rt_memheap_free(blocks[i]);
        CHECK(heap.available_size > 256, "fixture has insufficient total free capacity");
        reset_counts();
        allocated = rt_memheap_alloc(&heap, 256);
        CHECK(allocated == NULL, "fragmented fixture unexpectedly has a large block");
        CHECK(takes == 1 && releases == 1 && !held && !lock_errors,
              "fragmentation failure did not release the acquired lock");
    }
    else
    {
        CHECK(0, "unknown scenario");
    }
    printf("PASS %s takes=%u releases=%u held=%u errors=%u\n",
           scenario, takes, releases, held, lock_errors);
    return 0;
}

int main(int argc, char **argv)
{
    void *pool = pool_map();
    if (!pool || (uintptr_t)pool + 65536 > UINT32_MAX)
    {
        if (pool) pool_unmap(pool);
        puts("SKIP: could not reserve a pool below 4 GiB");
        return 77;
    }
    int result = exercise(argc > 1 ? argv[1] : "stale", pool);
    pool_unmap(pool);
    return result;
}
"""


class SmpMemheapTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = os.environ.get("SMP_TEST_CC") or shutil.which("gcc")
        if not compiler:
            raise unittest.SkipTest("Set SMP_TEST_CC to a native GCC compiler")
        cls.compiler = str(Path(compiler).resolve())
        cls.temp = tempfile.TemporaryDirectory(prefix="smp-memheap-")
        cls.addClassCleanup(cls.temp.cleanup)
        cls.work = Path(cls.temp.name)
        (cls.work / "rtconfig.h").write_text(CONFIG, encoding="utf-8")
        cls.harness = cls.work / "harness.c"
        cls.harness.write_text(HARNESS, encoding="utf-8")
        cls.executables = {}
        cls.environment = dict(os.environ)
        cls.environment["PATH"] = str(Path(cls.compiler).parent) + os.pathsep + os.environ.get("PATH", "")

    def executable(self, mutation=None):
        if mutation in self.executables:
            return self.executables[mutation]
        source = SOURCE
        if mutation:
            original = SOURCE.read_text(encoding="utf-8")
            start = original.index("__ROM_USED void *rt_memheap_alloc(")
            end = original.index("RTM_EXPORT(rt_memheap_alloc);", start)
            body = original[start:end]
            if mutation == "old_prelock_capacity":
                anchor = "result = rt_sem_take(&(heap->lock), RT_WAITING_FOREVER);"
                self.assertEqual(body.count(anchor), 1, "old guard injection needs one acquire")
                body = body.replace(anchor, "if (size >= heap->available_size) return RT_NULL;\n    " + anchor, 1)
            elif mutation == "missing_failure_unlock":
                anchor = "rt_sem_release(&(heap->lock));"
                self.assertGreaterEqual(body.count(anchor), 2, "need separate success/failure releases")
                head, tail = body.rsplit(anchor, 1)
                body = head + tail
            else:
                self.fail(f"unknown mutation: {mutation}")
            source = self.work / f"memheap_{mutation}.c"
            source.write_text(original[:start] + body + original[end:], encoding="utf-8")
        executable = self.work / ((mutation or "actual") + (".exe" if os.name == "nt" else ""))
        command = [self.compiler, "-std=c99", "-D_GNU_SOURCE", "-O0", "-ffunction-sections",
                   "-fdata-sections", "-Werror=implicit-function-declaration",
                   "-Wno-pointer-to-int-cast", "-Wno-int-to-pointer-cast",
                   "-I", str(self.work), "-I", str(HEADERS), str(self.harness), str(source),
                   "-Wl,--gc-sections", "-static-libgcc", "-o", str(executable)]
        result = subprocess.run(command, capture_output=True, text=True, timeout=60, env=self.environment)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.executables[mutation] = executable
        return executable

    def run_scenario(self, scenario, mutation=None):
        result = subprocess.run([str(self.executable(mutation)), scenario], capture_output=True,
                                text=True, timeout=10, env=self.environment)
        if result.returncode == 77:
            self.skipTest(result.stdout.strip())
        print(f"{mutation or 'actual'}/{scenario}: {result.stdout.strip()}")
        return result

    def test_acquire_refreshes_stale_capacity_before_allocation(self):
        result = self.run_scenario("stale")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("PASS stale", result.stdout)

    def test_real_capacity_and_fragmentation_failures_release_the_lock(self):
        for scenario in ("oom", "fragmented"):
            with self.subTest(scenario=scenario):
                result = self.run_scenario(scenario)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn(f"PASS {scenario}", result.stdout)

    def test_old_prelock_capacity_guard_fails_the_stale_case(self):
        result = self.run_scenario("stale", "old_prelock_capacity")
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn("stale capacity rejected before acquire takes=0 releases=0", result.stdout)

    def test_missing_failure_unlock_is_detected(self):
        for scenario in ("oom", "fragmented"):
            with self.subTest(scenario=scenario):
                result = self.run_scenario(scenario, "missing_failure_unlock")
                self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
                self.assertIn("did not release the acquired lock takes=1 releases=0 held=1", result.stdout)


if __name__ == "__main__":
    unittest.main()
