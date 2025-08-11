

#include "includes.h"
#include "os.h"
#include "fsl_os_abstraction.h"
#include "rtthread.h"
//#include <time.h>
#include "utils/common.h"
//#include "mbedtls/platform_util.h"

#define OS_USEC_OF_SEC      1000000U
#define OS_MSEC_OF_SEC      1000U
#define TICKS_TO_MSEC(tick) ((uint32_t)((uint64_t)(tick)*1000uL / (uint64_t)configTICK_RATE_HZ))

#if 0
/* returns time in micro-secs since time began */
static unsigned int os_timestamp(void)
{
    uint32_t nticks;
    uint32_t counter;

    vPortEnterCritical();
    nticks  = xTaskGetTickCount();
    counter = SysTick->VAL;

    /*
     * If this is called after SysTick counter
     * expired but before SysTick Handler got a
     * chance to run, then set the return value
     * to the start of next tick.
     */
    if ((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) != 0U)
    {
        nticks++;
        counter = CNTMAX;
    }

    vPortExitCritical();
    return ((CNTMAX - counter) / CPU_CLOCK_TICKSPERUSEC) + (nticks * USECSPERTICK);
}


int gettimeofday(struct timeval *tp, void *tzp)
{
    uint32_t sec, usec;
    uint32_t msecs = OSA_TimeGetMsec();

    sec = msecs / OS_MSEC_OF_SEC;

    usec = (msecs % OS_MSEC_OF_SEC) * OS_MSEC_OF_SEC;

    tp->tv_sec  = sec;
    tp->tv_usec = usec;

    return 0;
}
#endif

void os_sleep(os_time_t sec, os_time_t usec)
{
    if (sec)
    {
        OSA_TimeDelay(sec * OS_MSEC_OF_SEC);
    }
    if (usec)
    {
        OSA_TimeDelay(usec / OS_MSEC_OF_SEC);
    }
}

int os_get_time(struct os_time *t)
{
    TimeOut_t xCurrentTime = {0};

    uint64_t ullTickCount = 0ULL;

    uint64_t msecs;

    vPortEnterCritical();

    vTaskSetTimeOutState(&xCurrentTime);
    ullTickCount = (uint64_t)(xCurrentTime.xOverflowCount) << (sizeof(TickType_t) * 8);

    /* Add the current tick count. */
    ullTickCount += xCurrentTime.xTimeOnEntering;

    vPortExitCritical();

    msecs = TICKS_TO_MSEC(ullTickCount);

    t->sec = msecs / OS_MSEC_OF_SEC;
    // t->usec = (msecs % OS_MSEC_OF_SEC) * OS_MSEC_OF_SEC;
    t->usec = 0;
    return 0;
}

int os_get_reltime(struct os_reltime *t)
{
    return os_get_time((struct os_time *)t);
}

int os_mktime(int year, int month, int day, int hour, int min, int sec, os_time_t *t)
{
#if 0
	struct tm tm, *tm1;
	time_t t_local, t1, t2;
	os_time_t tz_offset;

	if (year < 1970 || month < 1 || month > 12 || day < 1 || day > 31 ||
	    hour < 0 || hour > 23 || min < 0 || min > 59 || sec < 0 ||
	    sec > 60) {
		return -1;
	}

	memset(&tm, 0, sizeof(tm));
	tm.tm_year = year - 1900;
	tm.tm_mon = month - 1;
	tm.tm_mday = day;
	tm.tm_hour = hour;
	tm.tm_min = min;
	tm.tm_sec = sec;

	t_local = mktime(&tm);

	/* figure out offset to UTC */
	tm1 = localtime(&t_local);
	if (tm1) {
		t1 = mktime(tm1);
		tm1 = gmtime(&t_local);
		if (tm1) {
			t2 = mktime(tm1);
			tz_offset = t2 - t1;
		} else {
			tz_offset = 0;
		}
	} else {
		tz_offset = 0;
	}

	*t = (os_time_t)t_local - tz_offset;
#endif
    return 0;
}

int os_gmtime(os_time_t t, struct os_tm *tm)
{
#if 0
 	struct tm *tm2;
	time_t t2 = t;

	tm2 = gmtime(&t2);
	if (tm2 == NULL) {
		return -1;
	}
	tm->sec = tm2->tm_sec;
	tm->min = tm2->tm_min;
	tm->hour = tm2->tm_hour;
	tm->day = tm2->tm_mday;
	tm->month = tm2->tm_mon + 1;
	tm->year = tm2->tm_year + 1900;
#endif
    return 0;
}

int os_daemonize(const char *pid_file)
{
    return -1;
}

void os_daemonize_terminate(const char *pid_file)
{
}

unsigned long os_random(void)
{
    return OSA_Rand();
}

int os_get_random(unsigned char *buf, size_t len)
{
    assert(buf != NULL);
    unsigned char *ptr = buf;
    unsigned long word;
    int copy_len;

    while (len > 0)
    {
        word     = os_random();
        copy_len = MIN(sizeof(word), len);
        memcpy(ptr, &word, copy_len);
        ptr += copy_len;
        len -= copy_len;
    }

    return 0;
}

char *os_rel2abs_path(const char *rel_path)
{
    return NULL; /* strdup(rel_path) can be used here */
}

int os_program_init(void)
{
    return 0;
}

void os_program_deinit(void)
{
}

int os_setenv(const char *name, const char *value, int overwrite)
{
    return -1;
}

int os_unsetenv(const char *name)
{
    return -1;
}

char *os_readfile(const char *name, size_t *len)
{
    return NULL;
}

#if 0
int os_fdatasync(FILE *stream)
{
    return 0;
}
#endif

char *os_strdup(const char *s)
{
    size_t len;
    char *d;

    len = os_strlen(s);
    d   = os_malloc(len + 1);
    if (d == NULL)
    {
        return NULL;
    }
    os_memcpy(d, s, len);
    d[len] = '\0';
    return d;
}

void *os_memdup(const void *src, size_t len)
{
    void *r = NULL;

    if (len == 0)
    {
        len++;
    }

    r = os_malloc(len);

    if (r && src)
    {
        os_memcpy(r, src, len);
    }
    return r;
}

void *os_malloc(size_t size)
{
    return OSA_MemoryAllocate(size);
}

void os_free(void *ptr)
{
    if (ptr != NULL)
    {
        OSA_MemoryFree(ptr);
        ptr = NULL;
    }
}

void *os_zalloc(size_t size)
{
    return os_malloc(size);
}

void *os_realloc(void *ptr, size_t newsize)
{
    void *p;

    if (newsize == 0)
    {
        os_free(ptr);
        return NULL;
    }

    if (ptr == NULL)
    {
        return os_malloc(newsize);
    }

    p = os_zalloc(newsize);

    if (p)
    {
        if (ptr != NULL)
        {
            memcpy(p, ptr, newsize);
            os_free(ptr);
        }
    }

    return p;
}

size_t os_strlcpy(char *dest, const char *src, size_t siz)
{
    const char *s = src;
    size_t left   = siz;

    if (left)
    {
        /* Copy string up to the maximum size of the dest buffer */
        while (--left != 0)
        {
            if ((*dest++ = *s++) == '\0')
            {
                break;
            }
        }
    }

    if (left == 0)
    {
        /* Not enough room for the string; force NUL-termination */
        if (siz != 0)
        {
            *dest = '\0';
        }
        while (*s++)
        {
            ; /* determine total src string length */
        }
    }

    return s - src - 1;
}

int os_exec(const char *program, const char *arg, int wait_completion)
{
    return -1;
}

/* Duplicate S, returning an identical malloc'd string.  */
char *__strdup(const char *s)
{
    size_t len = strlen(s) + 1;
    void *new  = os_malloc(len);

    if (new == NULL)
    {
        return NULL;
    }
    return (char *)memcpy(new, s, len);
}

int os_strcasecmp(const char *s1, const char *s2)
{
    /*
     * Ignoring case is not required for main functionality, so just use
     * the case sensitive version of the function.
     */
    return os_strcmp(s1, s2);
}

int os_strncasecmp(const char *s1, const char *s2, size_t n)
{
    /*
     * Ignoring case is not required for main functionality, so just use
     * the case sensitive version of the function.
     */
    return os_strncmp(s1, s2, n);
}
