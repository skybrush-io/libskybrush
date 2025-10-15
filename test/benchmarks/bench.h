/*
   IGraph library.
   Copyright (C) 2013-2024  The igraph development team <igraph@igraph.org>

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef IGRAPH_BENCH_H
#define IGRAPH_BENCH_H

#include <math.h> /* round */
#include <sys/resource.h> /* getrusage */
#include <sys/time.h> /* gettimeofday */
#include <unistd.h> /* sleep */

#include <skybrush/skybrush.h>

static inline void igraph_get_cpu_time(double* data)
{

    struct rusage self;
    struct timeval real;
    gettimeofday(&real, NULL);
    getrusage(RUSAGE_SELF, &self);
    data[0] = (double)real.tv_sec + 1e-6 * real.tv_usec; /* real */
    data[1] = (double)self.ru_utime.tv_sec + 1e-6 * self.ru_utime.tv_usec; /* user */
    data[2] = (double)self.ru_stime.tv_sec + 1e-6 * self.ru_stime.tv_usec; /* system */
}

#define BENCH_INIT(name)                      \
    do {                                      \
        printf("\n|> Benchmark: %s\n", name); \
        usleep(200000);                       \
    } while (0)

#define REPEAT(CODE, N)                       \
    do {                                      \
        int rep_i;                            \
        for (rep_i = 0; rep_i < N; ++rep_i) { \
            CODE;                             \
        }                                     \
    } while (0)

#define BENCH(NAME, ...)                                           \
    do {                                                           \
        double start[3], stop[3];                                  \
        double r, u, s;                                            \
        igraph_get_cpu_time(start);                                \
        {                                                          \
            __VA_ARGS__;                                           \
        }                                                          \
        igraph_get_cpu_time(stop);                                 \
        r = 1e-3 * round(1e3 * (stop[0] - start[0]));              \
        u = 1e-3 * round(1e3 * (stop[1] - start[1]));              \
        s = 1e-3 * round(1e3 * (stop[2] - start[2]));              \
        printf("| %-80s %5.3gs  %5.3gs  %5.3gs\n", NAME, r, u, s); \
    } while (0)

static inline void sb_trajectory_init_from_fixture(sb_trajectory_t* trajectory, const char* fname)
{
    FILE* fp;
    int fd;

    fp = fopen(fname, "rb");
    if (fp == 0) {
        perror(fname);
        abort();
    }

    fd = fileno(fp);
    if (fd < 0) {
        perror(fname);
        abort();
    }

    sb_trajectory_init_from_binary_file(trajectory, fd);

    fclose(fp);
}

#endif
