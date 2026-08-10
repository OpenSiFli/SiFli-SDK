/**
  ******************************************************************************
  * @file   sdio_func.h
  * @author AIC software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2018-2024 AICSemi Ltd. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _SDIO_FUNC_H_
#define _SDIO_FUNC_H_

/*
 * SDIO function devices
 */
struct sdio_func {
	void	(*irq_handler)(struct sdio_func *); /* IRQ callback */
	unsigned int	num;		/* function number *///add
	unsigned short		vendor;		/* vendor id */ //add
	unsigned short		device;		/* device id */ //add
	void *drv_priv;
};
#endif