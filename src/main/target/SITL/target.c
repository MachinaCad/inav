/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <getopt.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <signal.h>

#include <platform.h>
#include "target.h"

#include "fc/runtime_config.h"
#include "common/utils.h"
#include "scheduler/scheduler.h"
#include "drivers/system.h"
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_tcp.h"
#include "config/config_streamer.h"
#include "build/version.h"

#include "target/SITL/sim/realFlight.h"
#include "target/SITL/sim/xplane.h"
#include "target/SITL/sim/machina.h"

#include "target/SITL/serial_proxy.h"

// More dummys
const int timerHardwareCount = 0;
timerHardware_t timerHardware[1];
uint32_t SystemCoreClock = 500 * 1e6; // fake 500 MHz;
char _estack = 0 ;
char _Min_Stack_Size = 0;

static pthread_mutex_t mainLoopLock;
static SitlSim_e sitlSim = SITL_SIM_NONE;
static struct timespec start_time;

// Lockstep: virtual time from process start, in integer nanoseconds (see sitlLockstep* below).
static bool lockstepEnabled = false;
static uint64_t lockstepXplaneStepNs = 1000000;     // X-Plane only: --lockstep=dt_us, fixed
static uint64_t lockstepNowNs = 0;                  // the virtual clock, ns since process start
static bool lockstepMainParked = false;
static pthread_t lockstepMainThread;
static pthread_mutex_t lockstepMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t lockstepCond = PTHREAD_COND_INITIALIZER;
static uint8_t pwmMapping[MAX_MOTORS + MAX_SERVOS];
static uint8_t mappingCount = 0;
static bool useImu = false;
static char *simIp = NULL;
static int simPort = 0;

static char **c_argv;

static void printVersion(void) {
    fprintf(stderr, "INAV %d.%d.%d SITL (%s)\n", FC_VERSION_MAJOR, FC_VERSION_MINOR, FC_VERSION_PATCH_LEVEL, shortGitRevision);
}

static void cleanupAndExit(int code, bool shouldExit) {
    if (sitlSim == SITL_SIM_XPLANE) {
        simXPlaneClose();
    } else if (sitlSim == SITL_SIM_MACHINA) {
        simMachinaClose();
    } else if (sitlSim == SITL_SIM_REALFLIGHT) {
        simRealFlightClose();
    }
    pthread_mutex_destroy(&mainLoopLock);

    if (shouldExit) {
        exit(code);
    }
}

static void on_sigint(int sig) {
    UNUSED(sig);
    fprintf(stderr, "\n[SYSTEM] Caught SIGINT, exiting...\n");
    cleanupAndExit(0, true);
}

void systemInit(void) {
    printVersion();
    clock_gettime(CLOCK_MONOTONIC, &start_time);
    fprintf(stderr, "[SYSTEM] Init...\n");

#if !defined(__FreeBSD__)  && !defined(__APPLE__)
    pthread_attr_t thAttr;
    int policy = 0;

    pthread_attr_init(&thAttr);
    pthread_attr_getschedpolicy(&thAttr, &policy);
    pthread_setschedprio(pthread_self(), sched_get_priority_min(policy));
    pthread_attr_destroy(&thAttr);
#endif

    if (pthread_mutex_init(&mainLoopLock, NULL) != 0) {
        fprintf(stderr, "[SYSTEM] Unable to create mainLoop lock.\n");
        exit(1);
    }

    struct sigaction sa;
    sa.sa_handler = on_sigint;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, NULL);

    if (sitlSim != SITL_SIM_NONE) {
        fprintf(stderr, "[SIM] Waiting for connection...\n");
    }

    switch (sitlSim) {
        case SITL_SIM_REALFLIGHT:
            if (mappingCount > RF_MAX_PWM_OUTS) {
                fprintf(stderr, "[SIM] Mapping error. RealFligt supports a maximum of %i PWM outputs.", RF_MAX_PWM_OUTS);
                sitlSim = SITL_SIM_NONE;
                break;
            }
            if (simRealFlightInit(simIp, pwmMapping, mappingCount, useImu)) {
                fprintf(stderr, "[SIM] Connection with RealFlight successfully established.\n");
            } else {
                fprintf(stderr, "[SIM] Connection with RealFlight NOT established.\n");
            }
            break;
        case SITL_SIM_XPLANE:
            if (mappingCount > XP_MAX_PWM_OUTS) {
                fprintf(stderr, "[SIM] Mapping error. RealFligt supports a maximum of %i PWM outputs.", XP_MAX_PWM_OUTS);
                sitlSim = SITL_SIM_NONE;
                break;
            }
            if (simXPlaneInit(simIp, simPort, pwmMapping, mappingCount, useImu)) {
                fprintf(stderr, "[SIM] Connection with X-Plane successfully established.\n");
            } else {
                fprintf(stderr, "[SIM] Connection with X-PLane NOT established.\n");
            }
            break;
        case SITL_SIM_MACHINA:
            if (simMachinaInit(simIp, simPort, lockstepEnabled)) {
                fprintf(stderr, "[SIM] machina native backend connected (%s).\n", lockstepEnabled ? "lockstep" : "wall clock");
            } else {
                fprintf(stderr, "[SIM] machina native backend NOT connected.\n");
                exit(3);
            }
            break;
        default:
          fprintf(stderr, "[SIM] No interface specified. Configurator only.\n");
          break;
    }

    rescheduleTask(TASK_SERIAL, SITL_SERIAL_TASK_US);
}

bool parseMapping(char* mapStr)
{
    char *split = strtok(mapStr, ",");
    char numBuf[2];
    while(split)
    {
        if (strlen(split) != 6) {
            return false;
        }

        if (split[0] == 'M' || split[0] == 'S') {
            memcpy(numBuf, &split[1], 2);
            int pwmOut = atoi(numBuf);
            memcpy(numBuf, &split[4], 2);
            int rOut = atoi(numBuf);
            if (pwmOut < 0 || rOut < 1) {
                return false;
            }
            if (split[0] == 'M') {
                pwmMapping[rOut - 1] = pwmOut - 1;
                pwmMapping[rOut - 1] |= 0x80;
                mappingCount++;
            } else if (split[0] == 'S') {
                pwmMapping[rOut - 1] = pwmOut;
                mappingCount++;
            }
        } else {
            return false;
        }
        split = strtok(NULL, ",");
    }

    return true;
}

OptSerialStopBits_e parseStopBits(const char* optarg){
    if ( strcmp(optarg, "One") == 0 ) {
        return OPT_SERIAL_STOP_BITS_ONE;
    } else if ( strcmp(optarg, "Two") == 0 ) {
        return OPT_SERIAL_STOP_BITS_TWO;
    } else  {
        return OPT_SERIAL_STOP_BITS_INVALID;
    }
}

OptSerialParity_e parseParity(const char* optarg){
    if ( strcmp(optarg, "Even") == 0 ) {
        return OPT_SERIAL_PARITY_EVEN;
    } else if ( strcmp(optarg, "None") == 0 ) {
        return OPT_SERIAL_PARITY_NONE;
    } else if ( strcmp(optarg, "Odd") == 0 ) {
        return OPT_SERIAL_PARITY_ODD;
    } else  {
        return OPT_SERIAL_PARITY_INVALID;
    }
}

void printCmdLineOptions(void)
{
    printVersion();
    fprintf(stderr, "Avaiable options:\n");
    fprintf(stderr, "--path=[path]                  Path and filename of eeprom.bin. If not specified 'eeprom.bin' in program directory is used.\n");
    fprintf(stderr, "--sim=[rf|xp|machina]          Simulator interface: rf = RealFligt, xp = XPlane, machina = machina's native binary backend. Example: --sim=rf\n");
    fprintf(stderr, "--lockstep[=dt_us]             Lockstep: virtual time from process start, stepped by the simulator (machina, xp). xp steps by dt_us (default 1000); machina by each packet's step.\n");
    fprintf(stderr, "--simip=[ip]                   IP-Address oft the simulator host. If not specified localhost (127.0.0.1) is used.\n");
    fprintf(stderr, "--simport=[port]               Port oft the simulator host.\n");
    fprintf(stderr, "--useimu                       Use IMU sensor data from the simulator instead of using attitude data from the simulator directly (experimental, not recommended).\n");
    fprintf(stderr, "--serialuart=[uart]            UART number on which serial receiver is configured in SITL, f.e. 3 for UART3\n");
    fprintf(stderr, "--serialport=[serialport]      Host's serial port to which serial receiver/proxy FC is connected, f.e. COM3, /dev/ttyACM3\n");
    fprintf(stderr, "--baudrate=[baudrate]          Serial receiver baudrate (default: 115200).\n");
    fprintf(stderr, "--stopbits=[None|One|Two]      Serial receiver stopbits (default: One).\n");
    fprintf(stderr, "--parity=[Even|None|Odd]       Serial receiver parity (default: None).\n");
    fprintf(stderr, "--fcproxy                      Use inav/betaflight FC as a proxy for serial receiver.\n");
    fprintf(stderr, "--tcpbaseport=[port]           Base TCP port for UART sockets (default: 5760)\n");
    fprintf(stderr, "--chanmap=[mapstring]          Channel mapping. Maps INAVs motor and servo PWM outputs to the virtual receiver output in the simulator.\n");
    fprintf(stderr, "                               The mapstring has the following format: M(otor)|S(servo)<INAV-OUT>-<RECEIVER-OUT>,... All numbers must have two digits\n");
    fprintf(stderr, "                               For example: Map motor 1 to virtal receiver output 1, servo 1 to output 2 and servo 2 to output 3:\n");
    fprintf(stderr, "                               --chanmap=M01-01,S01-02,S02-03\n");
}

void parseArguments(int argc, char *argv[])
{
    // Stash these so we can rexec on reboot, just like a FC does
    c_argv = calloc(argc+1, sizeof(char *));
    for (int i = 0; i < argc; i++) {
        c_argv[i] = strdup(argv[i]);
    }
    int c;
    while(true) {
        static struct option longOpt[] = {
            {"sim", required_argument, 0, 's'},
            {"useimu", no_argument, 0, 'u'},
            {"chanmap", required_argument, 0, 'c'},
            {"simip", required_argument, 0, 'i'},
            {"simport", required_argument, 0, 'p'},
            {"help", no_argument, 0, 'h'},
            {"path", required_argument, 0, 'e'},
            {"version", no_argument, 0, 'v'},
            {"serialuart", required_argument, 0, '0'},
            {"serialport", required_argument, 0, '1'},
            {"baudrate", required_argument, 0, '2'},
            {"stopbits", required_argument, 0, '3'},
            {"parity", required_argument, 0, '4'},
            {"fcproxy", no_argument, 0, '5'},
            {"tcpbaseport", required_argument, 0, '6'},
            {"lockstep", optional_argument, 0, 'l'},
            {NULL, 0, NULL, 0}
        };

        c = getopt_long_only(argc, argv, "s:c:h", longOpt, NULL);
        if (c == -1)
            break;

        switch (c) {
            case 's':
                if (strcmp(optarg, "rf") == 0) {
                    sitlSim = SITL_SIM_REALFLIGHT;
                } else if (strcmp(optarg, "xp") == 0){
                    sitlSim = SITL_SIM_XPLANE;
                } else if (strcmp(optarg, "machina") == 0) {
                    sitlSim = SITL_SIM_MACHINA;
                } else {
                    fprintf(stderr, "[SIM] Unsupported simulator %s.\n", optarg);
                }
                break;

            case 'c':
                if (!parseMapping(optarg) && sitlSim != SITL_SIM_NONE) {
                    fprintf(stderr, "[SIM] Invalid channel mapping string.\n");
                    printCmdLineOptions();
                    exit(0);
                }
                break;
            case 'p':
                simPort = atoi(optarg);
                break;
            case 'l':
                lockstepEnabled = true;
                if (optarg) {
                    const long dtUs = atol(optarg);
                    if (dtUs <= 0 || dtUs > 1000000) {
                        fprintf(stderr, "[lockstep] Invalid step %s us\n", optarg);
                        exit(2);
                    }
                    lockstepXplaneStepNs = (uint64_t)dtUs * 1000;
                }
                break;
            case 'u':
                useImu = true;
                break;
            case 'i':
                simIp = optarg;
                break;
            case 'e':
                if (!configFileSetPath(optarg)){
                    fprintf(stderr, "[EEPROM] Invalid path, using eeprom file in program directory\n.");
                }
                break;
            case 'v':
                printVersion();
                exit(0);
            case '0':
                serialUartIndex = atoi(optarg);
                if ( (serialUartIndex<1) || (serialUartIndex>8) ) {
                    fprintf(stderr, "[serialuart] Invalid argument\n.");
                    exit(0);
                }
                break;
            case '1':
                if ( (strlen(optarg)<1) || (strlen(optarg)>63) ) {
                    fprintf(stderr, "[serialport] Invalid argument\n.");
                    exit(0);
                } else {
                    strcpy( serialPort, optarg );
                }
                break;
            case '2':
                serialBaudRate = atoi(optarg);
                if ( serialBaudRate < 1200 )
                {
                    fprintf(stderr, "[baudrate] Invalid argument\n.");
                    exit(0);
                }
                break;
            case '3':
                serialStopBits = parseStopBits(optarg);
                if ( serialStopBits == OPT_SERIAL_STOP_BITS_INVALID )
                {
                    fprintf(stderr, "[stopbits] Invalid argument\n.");
                    exit(0);
                }
                break;

            case '4':
                serialParity = parseParity(optarg);
                if ( serialParity== OPT_SERIAL_PARITY_INVALID )
                {
                    fprintf(stderr, "[parity] Invalid argument\n.");
                    exit(0);
                }
                break;
            case '5':
                serialFCProxy = true;
                break;
            case '6': {
                char *endptr = NULL;
                long basePort = strtol(optarg, &endptr, 10);
                if ((endptr == NULL) || (*endptr != '\0') || basePort <= 0 || basePort > UINT16_MAX || basePort + SERIAL_PORT_COUNT - 1 > UINT16_MAX) {
                    fprintf(stderr, "[tcpbaseport] Invalid argument\n.");
                    exit(0);
                }
                tcpBasePort = (uint16_t)basePort;
                break;
            }

            default:
                printCmdLineOptions();
                exit(0);
        }
    }

    if (simIp == NULL) {
        simIp = malloc(10);
        strcpy(simIp, "127.0.0.1");
    }

    // Lockstep needs a simulator that steps the clock: the native machina backend or X-Plane.
    if (lockstepEnabled && sitlSim != SITL_SIM_MACHINA && sitlSim != SITL_SIM_XPLANE) {
        fprintf(stderr, "[lockstep] --lockstep needs --sim=machina or --sim=xp\n");
        exit(2);
    }
    lockstepMainThread = pthread_self();
}


bool lockMainPID(void) {
    return pthread_mutex_trylock(&mainLoopLock) == 0;
}

void unlockMainPID(void)
{
    pthread_mutex_unlock(&mainLoopLock);
}

// Replacements for system functions
static timeUs_t realMicros(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    return (now.tv_sec - start_time.tv_sec) * 1000000 + (now.tv_nsec - start_time.tv_nsec) / 1000;
}

timeUs_t micros(void) {
    if (lockstepEnabled) {
        return __atomic_load_n(&lockstepNowNs, __ATOMIC_ACQUIRE) / 1000;
    }
    return realMicros();
}

/*
 * Lockstep (machina, closed-loop-plan.md D3).
 *
 * With --lockstep the firmware's clock is virtual from process start (t = 0) and advances only
 * when the simulator thread applies a sensor packet; it never reads the host's clock. Each step
 * runs in a fixed order: sensors(k), then the clock to the end of the step, then the main loop
 * until nothing is due, then motors(k). The main thread runs only while the clock stands still,
 * and the simulator thread changes the clock and the simulated sensors only while the main thread
 * is parked: in the scheduler with no task due, or in delay(). Init is driven by sensor packets
 * too: its delays park the main thread until enough steps have passed. Nothing in a run depends on
 * when packets arrive, only on their contents and order.
 */
bool sitlLockstepEnabled(void) { return lockstepEnabled; }

bool sitlLockstepActive(void) { return lockstepEnabled; }

uint64_t sitlLockstepNowNs(void) { return __atomic_load_n(&lockstepNowNs, __ATOMIC_ACQUIRE); }

uint64_t sitlLockstepXplaneStepNs(void) { return lockstepXplaneStepNs; }

static bool lockstepOnMainThread(void)
{
    return lockstepEnabled && pthread_equal(pthread_self(), lockstepMainThread);
}

// Main thread: park until the virtual clock reaches targetNs.
static void lockstepParkUntil(uint64_t targetNs)
{
    pthread_mutex_lock(&lockstepMutex);
    while (lockstepNowNs < targetNs) {
        lockstepMainParked = true;
        pthread_cond_broadcast(&lockstepCond);
        pthread_cond_wait(&lockstepCond, &lockstepMutex);
    }
    lockstepMainParked = false;
    pthread_mutex_unlock(&lockstepMutex);
}

// Main thread, from the scheduler when no task is due: park until the next step.
void sitlLockstepIdle(void)
{
    pthread_mutex_lock(&lockstepMutex);
    const uint64_t targetNs = lockstepNowNs + 1;
    pthread_mutex_unlock(&lockstepMutex);
    lockstepParkUntil(targetNs);
}

// Simulator thread, before it touches the sensors or the clock: wait until the main thread is
// parked (at boot, until init reaches its first delay or the scheduler).
void sitlLockstepWaitParked(void)
{
    pthread_mutex_lock(&lockstepMutex);
    while (!lockstepMainParked) {
        pthread_cond_wait(&lockstepCond, &lockstepMutex);
    }
    pthread_mutex_unlock(&lockstepMutex);
}

// Simulator thread, after the sensors of a step are applied: move the clock to the end of the
// step, then wait until the main loop has run everything due by then and parked again.
void sitlLockstepAdvanceTo(uint64_t nowNs)
{
    pthread_mutex_lock(&lockstepMutex);
    __atomic_store_n(&lockstepNowNs, nowNs, __ATOMIC_RELEASE);
    lockstepMainParked = false;
    pthread_cond_broadcast(&lockstepCond);
    while (!lockstepMainParked) {
        pthread_cond_wait(&lockstepCond, &lockstepMutex);
    }
    pthread_mutex_unlock(&lockstepMutex);
}

uint64_t microsISR(void)
{
    return micros();
}

uint32_t millis(void) {
    return (uint32_t)(micros() / 1000);
}

void delayMicroseconds(timeUs_t us)
{
    if (lockstepOnMainThread()) {
        // Virtual time: the main thread parks until the simulator has stepped past the delay.
        lockstepParkUntil(sitlLockstepNowNs() + (uint64_t)us * 1000);
        return;
    }
    usleep(us);
}

void delay(timeMs_t ms)
{
    delayMicroseconds(ms * 1000UL);
}

void systemReset(void)
{
    fprintf(stderr, "[SYSTEM] Reset\n");
    cleanupAndExit(0, false);
#if defined(__CYGWIN__) || defined(__APPLE__) || GCC_MAJOR < 12
    for(int j = 3; j < 1024; j++) {
        close(j);
    }
#else
    closefrom(3);
#endif
    serialProxyClose();
    execvp(c_argv[0], c_argv); // restart
}

void systemResetToBootloader(void)
{
    fprintf(stderr, "[SYSTEM] Reset to bootloader\n");
    cleanupAndExit(0, true);
}

void failureMode(failureMode_e mode) {
    fprintf(stderr, "[SYSTEM] Failure mode %d\n", mode);
    while (true) {
        delay(1000);
    };
}

// Even more dummys and stubs
uint32_t getEscUpdateFrequency(void)
{
    return 400;
}

pwmInitError_e getPwmInitError(void)
{
    return PWM_INIT_ERROR_NONE;
}

const char *getPwmInitErrorMessage(void)
{
    return "No error";
}

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    UNUSED(io);
    UNUSED(cfg);
}

void timerInit(void) {
    // NOP
}

bool isMPUSoftReset(void)
{
    return false;
}

// Not in linux libs, but in arm-none-eabi ?!?
// https://github.com/lattera/freebsd/blob/master/lib/libc/string/strnstr.c
char * strnstr(const char *s, const char *find, size_t slen)
{
	char c, sc;
	size_t len;

	if ((c = *find++) != '\0') {
		len = strlen(find);
		do {
			do {
				if (slen-- < 1 || (sc = *s++) == '\0')
					return (NULL);
			} while (sc != c);
			if (len > slen)
				return (NULL);
		} while (strncmp(s, find, len) != 0);
		s--;
	}
	return ((char *)s);
}

int lookupAddress (char *name, int port, int type, struct sockaddr *addr, socklen_t* len )
{
    struct addrinfo *servinfo, *p;
    struct addrinfo hints = {.ai_family = AF_UNSPEC, .ai_socktype = type, .ai_flags = AI_V4MAPPED|AI_ADDRCONFIG};
    if (name == NULL) {
	hints.ai_flags |= AI_PASSIVE;
    }
  /*
    This nonsense is to uniformly deliver the same sa_family regardless of whether
    name is NULL or non-NULL ** ON LINUX **
    Otherwise, at least on Linux, we get
    - V6,V4 for the non-null case and
    - V4,V6 for the null case, regardless of gai.conf
    Which may confuse consumers
    FreeBSD and Windows behave consistently, giving V6 for Ipv6 enabled stacks
    unless a quad dotted address is specified (or a name resolveds to V4,
    or system policy enforces IPv4 over V6
  */
    struct addrinfo *p4 = NULL;
    struct addrinfo *p6 = NULL;

    int result;
    char aport[16];
    snprintf(aport, sizeof(aport), "%d", port);

    if ((result = getaddrinfo(name, aport, &hints, &servinfo)) != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(result));
        return result;
    } else {
	for(p = servinfo; p != NULL; p = p->ai_next) {
	    if(p->ai_family == AF_INET6)
		p6 = p;
	    else if(p->ai_family == AF_INET)
		p4 = p;
	}

	if (p6 != NULL)
	    p = p6;
	else if (p4 != NULL)
	    p = p4;
	else
	    return -1;
	memcpy(addr, p->ai_addr, p->ai_addrlen);
	*len = p->ai_addrlen;
	freeaddrinfo(servinfo);
    }
    return 0;
}

char *prettyPrintAddress(struct sockaddr* p, char *outbuf, size_t buflen)
{
    if (buflen < IPADDRESS_PRINT_BUFLEN) {
	return NULL;
    }
    char *bufp = outbuf;
    void *addr;
    uint16_t port;
    if (p->sa_family == AF_INET6) {
	struct sockaddr_in6 * ip = (struct sockaddr_in6*)p;
	addr = &ip->sin6_addr;
	port = ntohs(ip->sin6_port);
    } else {
	struct sockaddr_in * ip = (struct sockaddr_in*)p;
	port = ntohs(ip->sin_port);
	addr = &ip->sin_addr;
    }
    const char *res = inet_ntop(p->sa_family, addr, outbuf+1, buflen-1);
    if (res != NULL) {
	char *ptr = (char*)res+strlen(res);
	if (p->sa_family == AF_INET6) {
	    *bufp ='[';
	    *ptr++ = ']';
	} else {
	    bufp++;
	}
	sprintf(ptr, ":%d", port);
	return bufp;
    }
    return NULL;
}
