/*******************************************************************************
Copyright (C) 2015 Annapurna Labs Ltd.

This file may be licensed under the terms of the Annapurna Labs Commercial
License Agreement.

Alternatively, this file can be distributed under the terms of the GNU General
Public License V2 as published by the Free Software Foundation and can be
found at http://www.gnu.org/licenses/gpl-2.0.html

Alternatively, redistribution and use in source and binary forms, with or
without modification, are permitted provided that the following conditions are
met:

    *     Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

    *     Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in
the documentation and/or other materials provided with the
distribution.

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

/**
 * @defgroup group_common HAL Common Layer
 * Includes all common header files used by HAL
 *  @{
 * @file   al_hal_common.h
 *
 */

#ifndef __AL_HAL_COMMON_H__
#define __AL_HAL_COMMON_H__

#include "al_hal_plat_types.h"
#include "al_hal_plat_services.h"

#include "al_hal_types.h"
#include "al_hal_reg_utils.h"

/* Get the maximal value out of two typed values */
#define al_max_t(type, x, y) ({		\
	type __max1 = (x);			\
	type __max2 = (y);			\
	__max1 > __max2 ? __max1 : __max2; })

/* Get the minimal value out of two typed values */
#define al_min_t(type, x, y) ({		\
	type __min1 = (x);			\
	type __min2 = (y);			\
	__min1 < __min2 ? __min1 : __min2; })

/* Get the number of elements in an array */
#define AL_ARR_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

/* Get the offset of a member of a struct */
#define al_offsetof(st, m) ((size_t)(&((st *)0)->m))

/* Get the pointer of the container structure of given pointer */
#define al_container_of(ptr, type, member) ({ \
		const typeof(((type *)0)->member) * __mptr = (ptr); \
		(type *)((uintptr_t)__mptr - al_offsetof(type, member)); \
	})

/* Count number of 1-bits */
#ifndef al_popcount
#define al_popcount(x)		__builtin_popcount(x)
#endif

/** @} end of Common group */
#endif				/* __AL_HAL_COMMON_H__ */
