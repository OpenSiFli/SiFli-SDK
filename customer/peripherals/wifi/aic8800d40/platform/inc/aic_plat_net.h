/**
  ******************************************************************************
  * @file   aic_plat_net.h
  * @author AIC software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2018-2025 AICSemi Ltd. All rights reserved.
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

#ifndef _AIC_PLAT_NET_H_
#define _AIC_PLAT_NET_H_

#include "lwip/netifapi.h"
#include "lwip/tcpip.h"
#include "lwip/netif.h"
#include "netif/etharp.h"
#include "netif/ethernetif.h"

// lwip opts sanity check
#if !(LWIP_HAVE_LOOPIF & LWIP_NETIF_LOOPBACK)
#error "loopback netif required"
#endif

#endif /* _AIC_PLAT_NET_H_ */
