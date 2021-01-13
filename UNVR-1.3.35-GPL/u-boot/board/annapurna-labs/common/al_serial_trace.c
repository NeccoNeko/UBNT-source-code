/*******************************************************************************
Copyright (C) 2016 Annapurna Labs Ltd.

This software file is triple licensed: you can use it either under the terms of
Commercial, the GPL, or the BSD license, at your option.

a) If you received this File from Annapurna Labs and you have entered into a
   commercial license agreement (a "Commercial License") with Annapurna Labs,
   the File is licensed to you under the terms of the applicable Commercial
   License.

Alternatively,

b) This file is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License as
   published by the Free Software Foundation; either version 2 of the
   License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public
   License along with this library; if not, write to the Free
   Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston,
   MA 02110-1301 USA

Alternatively,

c) Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

    *   Redistributions of source code must retain the above copyright notice,
	this list of conditions and the following disclaimer.

    *   Redistributions in binary form must reproduce the above copyright
	notice, this list of conditions and the following disclaimer in the
	documentation and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/
#include <al_serial_trace.h>
#include <al_trace.h>
#include <common.h>
#include <libfdt_env.h>
#include <fdt_support.h>
#include <libfdt.h>
#include <fdtdec.h>

DECLARE_GLOBAL_DATA_PTR;

static struct al_trace_handle trace_tmp;
static struct al_trace_handle trace_final;
static struct al_trace_handle *trace;
static unsigned int disabled = 1;

static unsigned int time_stamp_str_get(char *s, unsigned int len)
{
	unsigned int i;

	i = snprintf(s, len, "%010u", (unsigned int)timer_get_us());
	if (i < len)
		s[i++] = 'u';
	if (i < len)
		s[i++] = 's';
	if (i < len)
		s[i++] = ':';
	if (i < len)
		s[i++] = ' ';

	return i;
}

void al_serial_trace_init_early(void)
{
	trace = &trace_tmp;
	al_trace_init(trace,
		(void *)(uintptr_t)CONFIG_AL_TRACE_TMP_BUFF_ADDR, CONFIG_AL_TRACE_TMP_BUFF_SIZE,
		AL_FALSE, time_stamp_str_get);
	disabled = 0;
	printf("Trace early: %p, %x\n",
		(void *)(uintptr_t)CONFIG_AL_TRACE_TMP_BUFF_ADDR, CONFIG_AL_TRACE_TMP_BUFF_SIZE);
}

void al_serial_trace_init_dt_based(void)
{
	struct al_trace_handle *trace_prev = trace;
	char *start = NULL;
	unsigned int size;
	int off;

	disabled = 1;

	off = fdt_path_offset(working_fdt, "/soc/trace");
	if (off >= 0) {
		const struct fdt_property *fdt_prop;
		int len;

		fdt_prop = fdt_get_property(working_fdt, off, "reg", &len);
		if (fdt_prop) {
			const uint32_t *cell;

			cell = (uint32_t *)fdt_prop->data;
			len /= sizeof(uint32_t);

			if (len == 4) {
				start = (char *)(uintptr_t)fdt32_to_cpu(cell[1]);
				size = fdt32_to_cpu(cell[3]);
			} else {
				printf("Trace 'reg' property invalid!\n");
				return;
			}
		} else {
			printf("Trace 'reg' property not found!\n");
			return;
		}
	} else {
		printf("Trace disabled\n");
		return;
	}

	trace = &trace_final;
	al_trace_init(trace, start, size, AL_TRUE, time_stamp_str_get);
	if (trace_prev)
		al_trace_append(trace, trace_prev);
	disabled = 0;
	printf("Trace final: %p, %x\n", start, size);
}

void al_serial_trace_putchar(int c)
{
	if (!(gd->flags & GD_FLG_RELOC))
		return;

	if (disabled)
		return;

	al_trace_putchar(trace, c);
}

void al_serial_trace_dump(void)
{
	if (disabled) {
		printf("Trace disabled\n");
		return;
	}

	al_trace_dump(trace, 0);
}
