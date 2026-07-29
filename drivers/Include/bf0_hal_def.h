/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BF0_HAL_DEF
#define __BF0_HAL_DEF

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup COMMON Common
  * @brief Common HAL common type definition
  * @{
  */


/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <stdbool.h>
/* Exported types ------------------------------------------------------------*/

#ifndef WIN32
#define _FOREACH_FUNC_0(fn, pad, a)
#define _FOREACH_FUNC_1(fn, pad, a)       fn(pad, a)
#define _FOREACH_FUNC_2(fn, pad, a, ...)  fn(pad, a), _FOREACH_FUNC_1(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_3(fn, pad, a, ...)  fn(pad, a), _FOREACH_FUNC_2(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_4(fn, pad, a, ...)  fn(pad, a), _FOREACH_FUNC_3(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_5(fn, pad, a, ...)  fn(pad, a), _FOREACH_FUNC_4(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_6(fn, pad, a, ...)  fn(pad, a), _FOREACH_FUNC_5(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_7(fn, pad, a, ...)  fn(pad, a), _FOREACH_FUNC_6(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_8(fn, pad, a, ...)  fn(pad, a), _FOREACH_FUNC_7(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_9(fn, pad, a, ...)  fn(pad, a), _FOREACH_FUNC_8(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_10(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_9(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_11(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_10(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_12(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_11(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_13(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_12(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_14(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_13(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_15(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_14(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_16(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_15(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_17(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_16(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_18(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_17(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_19(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_18(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_20(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_19(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_21(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_20(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_22(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_21(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_23(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_22(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_24(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_23(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_25(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_24(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_26(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_25(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_27(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_26(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_28(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_27(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_29(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_28(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_30(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_29(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_31(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_30(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_32(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_31(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_33(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_32(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_34(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_33(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_35(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_34(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_36(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_35(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_37(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_36(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_38(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_37(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_39(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_38(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_40(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_39(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_41(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_40(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_42(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_41(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_43(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_42(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_44(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_43(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_45(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_44(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_46(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_45(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_47(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_46(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_48(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_47(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_49(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_48(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_50(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_49(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_51(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_50(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_52(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_51(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_53(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_52(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_54(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_53(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_55(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_54(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_56(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_55(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_57(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_56(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_58(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_57(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_59(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_58(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_60(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_59(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_61(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_60(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_62(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_61(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_63(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_62(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_64(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_63(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_65(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_64(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_66(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_65(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_67(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_66(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_68(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_67(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_69(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_68(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_70(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_69(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_71(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_70(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_72(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_71(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_73(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_72(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_74(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_73(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_75(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_74(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_76(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_75(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_77(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_76(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_78(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_77(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_79(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_78(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_80(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_79(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_81(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_80(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_82(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_81(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_83(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_82(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_84(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_83(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_85(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_84(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_86(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_85(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_87(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_86(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_88(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_87(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_89(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_88(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_90(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_89(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_91(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_90(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_92(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_91(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_93(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_92(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_94(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_93(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_95(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_94(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_96(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_95(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_97(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_96(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_98(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_97(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_99(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_98(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_100(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_99(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_101(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_100(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_102(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_101(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_103(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_102(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_104(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_103(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_105(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_104(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_106(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_105(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_107(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_106(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_108(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_107(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_109(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_108(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_110(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_109(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_111(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_110(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_112(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_111(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_113(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_112(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_114(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_113(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_115(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_114(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_116(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_115(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_117(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_116(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_118(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_117(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_119(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_118(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_120(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_119(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_121(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_120(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_122(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_121(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_123(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_122(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_124(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_123(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_125(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_124(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_126(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_125(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_127(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_126(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_128(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_127(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_129(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_128(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_130(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_129(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_131(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_130(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_132(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_131(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_133(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_132(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_134(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_133(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_135(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_134(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_136(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_135(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_137(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_136(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_138(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_137(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_139(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_138(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_140(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_139(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_141(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_140(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_142(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_141(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_143(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_142(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_144(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_143(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_145(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_144(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_146(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_145(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_147(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_146(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_148(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_147(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_149(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_148(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_150(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_149(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_151(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_150(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_152(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_151(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_153(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_152(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_154(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_153(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_155(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_154(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_156(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_155(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_157(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_156(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_158(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_157(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_159(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_158(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_160(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_159(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_161(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_160(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_162(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_161(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_163(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_162(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_164(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_163(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_165(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_164(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_166(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_165(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_167(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_166(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_168(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_167(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_169(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_168(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_170(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_169(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_171(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_170(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_172(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_171(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_173(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_172(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_174(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_173(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_175(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_174(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_176(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_175(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_177(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_176(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_178(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_177(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_179(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_178(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_180(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_179(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_181(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_180(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_182(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_181(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_183(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_182(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_184(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_183(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_185(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_184(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_186(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_185(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_187(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_186(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_188(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_187(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_189(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_188(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_190(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_189(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_191(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_190(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_192(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_191(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_193(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_192(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_194(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_193(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_195(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_194(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_196(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_195(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_197(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_196(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_198(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_197(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_199(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_198(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_200(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_199(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_201(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_200(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_202(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_201(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_203(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_202(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_204(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_203(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_205(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_204(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_206(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_205(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_207(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_206(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_208(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_207(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_209(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_208(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_210(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_209(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_211(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_210(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_212(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_211(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_213(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_212(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_214(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_213(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_215(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_214(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_216(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_215(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_217(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_216(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_218(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_217(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_219(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_218(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_220(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_219(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_221(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_220(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_222(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_221(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_223(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_222(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_224(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_223(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_225(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_224(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_226(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_225(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_227(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_226(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_228(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_227(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_229(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_228(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_230(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_229(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_231(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_230(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_232(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_231(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_233(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_232(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_234(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_233(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_235(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_234(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_236(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_235(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_237(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_236(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_238(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_237(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_239(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_238(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_240(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_239(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_241(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_240(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_242(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_241(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_243(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_242(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_244(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_243(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_245(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_244(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_246(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_245(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_247(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_246(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_248(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_247(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_249(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_248(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_250(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_249(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_251(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_250(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_252(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_251(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_253(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_252(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_254(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_253(fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_255(fn, pad, a, ...) fn(pad, a), _FOREACH_FUNC_254(fn, pad, __VA_ARGS__)

/**
 * @brief  Get variant arguments number, support maximum 255 arguments
 *
 * @return arguments number
 */
#define _GET_ARG_( \
    _1,_2,_3,_4,_5,_6,_7,_8,_9,_10,_11,_12,_13,_14,_15,_16,_17,_18,_19,_20, \
    _21,_22,_23,_24,_25,_26,_27,_28,_29,_30,_31,_32,_33,_34,_35,_36,_37,_38,_39,_40, \
    _41,_42,_43,_44,_45,_46,_47,_48,_49,_50,_51,_52,_53,_54,_55,_56,_57,_58,_59,_60, \
    _61,_62,_63,_64,_65,_66,_67,_68,_69,_70,_71,_72,_73,_74,_75,_76,_77,_78,_79,_80, \
    _81,_82,_83,_84,_85,_86,_87,_88,_89,_90,_91,_92,_93,_94,_95,_96,_97,_98,_99,_100, \
    _101,_102,_103,_104,_105,_106,_107,_108,_109,_110,_111,_112,_113,_114,_115,_116,_117,_118,_119,_120, \
    _121,_122,_123,_124,_125,_126,_127,_128,_129,_130,_131,_132,_133,_134,_135,_136,_137,_138,_139,_140, \
    _141,_142,_143,_144,_145,_146,_147,_148,_149,_150,_151,_152,_153,_154,_155,_156,_157,_158,_159,_160, \
    _161,_162,_163,_164,_165,_166,_167,_168,_169,_170,_171,_172,_173,_174,_175,_176,_177,_178,_179,_180, \
    _181,_182,_183,_184,_185,_186,_187,_188,_189,_190,_191,_192,_193,_194,_195,_196,_197,_198,_199,_200, \
    _201,_202,_203,_204,_205,_206,_207,_208,_209,_210,_211,_212,_213,_214,_215,_216,_217,_218,_219,_220, \
    _221,_222,_223,_224,_225,_226,_227,_228,_229,_230,_231,_232,_233,_234,_235,_236,_237,_238,_239,_240, \
    _241,_242,_243,_244,_245,_246,_247,_248,_249,_250,_251,_252,_253,_254,_255,N,...) N

/**
 * @brief  Count variant arguments number, support maximum 255 arguments
 *
 * @param __va_args variant arguments
 * @return arguments number
 */
#define COUNT_ARGS(...) _GET_ARG_(__VA_ARGS__, \
    255,254,253,252,251,250,249,248,247,246,245,244,243,242,241,240,239,238,237,236,235,234,233,232,231,\
    230,229,228,227,226,225,224,223,222,221,220,219,218,217,216,215,214,213,212,211,210,209,208,207,206,\
    205,204,203,202,201,200,199,198,197,196,195,194,193,192,191,190,189,188,187,186,185,184,183,182,181,\
    180,179,178,177,176,175,174,173,172,171,170,169,168,167,166,165,164,163,162,161,160,159,158,157,156,\
    155,154,153,152,151,150,149,148,147,146,145,144,143,142,141,140,139,138,137,136,135,134,133,132,131,\
    130,129,128,127,126,125,124,123,122,121,120,119,118,117,116,115,114,113,112,111,110,109,108,107,106,\
    105,104,103,102,101,100,99,98,97,96,95,94,93,92,91,90,89,88,87,86,85,84,83,82,81,80,79,78,77,76,75, \
    74,73,72,71,70,69,68,67,66,65,64,63,62,61,60,59,58,57,56,55,54,53,52,51,50,49,48,47,46,45,44,43,42, \
    41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,\
    7,6,5,4,3,2,1,0)

/**
 * @brief  fn is called for each argument in __var_args__
 *
 *  e.g. `FOREACH_FUNC(fn, pad, a, b, c)` would be expanded as
 * ```
 *  fn(pad, a),
 *  fn(pad, b),
 *  fn(pad, c),
 * ```
 *
 * @param fn function to be called for each loop
 * @param pad argument to be passed to function fn
 * @param __va_args variant arguments
 * @return arguments number
 */
#define FOREACH_FUNC(fn, pad, ...)                                   \
    _FOREACH_FUNC_I(COUNT_ARGS(__VA_ARGS__), fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_I(N, fn, pad, ...)                             \
    _FOREACH_FUNC_II(N, fn, pad, __VA_ARGS__)
#define _FOREACH_FUNC_II(N, fn, pad, ...)                            \
    _FOREACH_FUNC_##N(fn, pad, __VA_ARGS__)


#endif /* !WIN32 */

#define HAL_CONCAT_2_(p1, p2)     p1##p2

/**@brief Concatenates two parameters.
 *
 * It realizes two level expansion to make it sure that all the parameters
 * are actually expanded before gluing them together.
 *
 * @param p1 First parameter to concatenating
 * @param p2 Second parameter to concatenating
 *
 * @return Two parameters glued together.
 *         They have to create correct C mnemonic in other case
 *         preprocessor error would be generated.
 *
 */
#define HAL_CONCAT_2(p1, p2)      HAL_CONCAT_2_(p1, p2)

/**
  * @brief  HAL Status structures definition
  */
typedef enum
{
    HAL_OK       = 0x00, /**< No error */
    HAL_ERROR    = 0x01, /**< General error */
    HAL_BUSY     = 0x02, /**< Busy */
    HAL_TIMEOUT  = 0x03,  /**< Timeout */

    HAL_EPIC_NOTHING_TO_DO = 0x10
} HAL_StatusTypeDef;

/**
  * @brief  HAL Lock structures definition
  */
typedef enum
{
    HAL_UNLOCKED = 0x00,
    HAL_LOCKED   = 0x01
} HAL_LockTypeDef;

#ifndef WIN32
#include "register.h"
#endif

/* Exported macros -----------------------------------------------------------*/

#define HAL_MEM_TYPE_NOR_FLASH       0U
#define HAL_MEM_TYPE_NAND_FLASH      1U
#define HAL_MEM_TYPE_SDMMC_STORAGE   2U
#define HAL_MEM_TYPE_PSRAM           3U
#define HAL_MEM_TYPE_UNKNOWN         0xFFU

#define UNUSED(X) (void)X      /* To avoid gcc/g++ warnings */

#define HAL_MAX_DELAY      0xFFFFFFFFU

#define HAL_IS_BIT_SET(REG, BIT)         (((REG) & (BIT)) == (BIT))
#define HAL_IS_BIT_CLR(REG, BIT)         (((REG) & (BIT)) == 0U)

#define __HAL_LINKDMA(__HANDLE__, __PPP_DMA_FIELD__, __DMA_HANDLE__)             \
                        do{                                                      \
                            (__HANDLE__)->__PPP_DMA_FIELD__ = &(__DMA_HANDLE__); \
                            (__DMA_HANDLE__).Parent = (__HANDLE__);              \
                        } while(0)

/** @brief Reset the Handle's State field.
  * @param  \__HANDLE__: specifies the Peripheral Handle.
  * @note  This macro can be used for the following purpose:
  *          - When the Handle is declared as local variable; before passing it as parameter
  *            to HAL_PPP_Init() for the first time, it is mandatory to use this macro
  *            to set to 0 the Handle's "State" field.
  *            Otherwise, "State" field may have any random value and the first time the function
  *            HAL_PPP_Init() is called, the low level hardware initialization will be missed
  *            (i.e. HAL_PPP_MspInit() will not be executed).
  *          - When there is a need to reconfigure the low level hardware: instead of calling
  *            HAL_PPP_DeInit() then HAL_PPP_Init(), user can make a call to this macro then HAL_PPP_Init().
  *            In this later function, when the Handle's "State" field is set to 0, it will execute the function
  *            HAL_PPP_MspInit() which will reconfigure the low level hardware.
  * @retval None
  */
#define __HAL_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = 0)

#define __HAL_LOCK(__HANDLE__)                                           \
                                do{                                        \
                                    if((__HANDLE__)->Lock == HAL_LOCKED)   \
                                    {                                      \
                                       return HAL_BUSY;                    \
                                    }                                      \
                                    else                                   \
                                    {                                      \
                                       (__HANDLE__)->Lock = HAL_LOCKED;    \
                                    }                                      \
                                  }while (0)

#define __HAL_UNLOCK(__HANDLE__)                                          \
                                  do{                                       \
                                      (__HANDLE__)->Lock = HAL_UNLOCKED;    \
                                    }while (0)

#if defined ( __GNUC__ ) && !defined (__CC_ARM) /* GNU Compiler */
#ifndef __weak
#define __weak   __attribute__((weak))
#endif /* __weak */
#ifndef __packed
#define __packed __attribute__((__packed__))
#endif /* __packed */
#endif /* __GNUC__ */


/* Macro to get variable aligned on 4-bytes, for __ICCARM__ the directive "#pragma data_alignment=4" must be used instead */
#if defined ( __GNUC__ ) && !defined (__CC_ARM) /* GNU Compiler */
#ifndef __ALIGN_END
#define __ALIGN_END    __attribute__ ((aligned (4)))
#endif /* __ALIGN_END */
#ifndef __ALIGN_BEGIN
#define __ALIGN_BEGIN
#endif /* __ALIGN_BEGIN */
#else
#ifndef __ALIGN_END
#define __ALIGN_END
#endif /* __ALIGN_END */
#ifndef __ALIGN_BEGIN
#if defined   (__CC_ARM)      /* ARM Compiler */
#define __ALIGN_BEGIN    __align(4)
#elif defined (__ICCARM__)    /* IAR Compiler */
#define __ALIGN_BEGIN
#endif /* __CC_ARM */
#endif /* __ALIGN_BEGIN */
#endif /* __GNUC__ */

/**
  * @brief  __RAM_FUNC definition
  */
#if defined ( __CC_ARM   )
/* ARM Compiler
   ------------
   RAM functions are defined using the toolchain options.
   Functions that are executed in RAM should reside in a separate source module.
   Using the 'Options for File' dialog you can simply change the 'Code / Const'
   area of a module to a memory space in physical RAM.
   Available memory areas are declared in the 'Target' tab of the 'Options for Target'
   dialog.
*/
#define __RAM_FUNC HAL_StatusTypeDef

#elif defined ( __ICCARM__ )
/* ICCARM Compiler
   ---------------
   RAM functions are defined using a specific toolchain keyword "__ramfunc".
*/
#define __RAM_FUNC __ramfunc HAL_StatusTypeDef

#elif defined   (  __GNUC__  )
/* GNU Compiler
   ------------
  RAM functions are defined using a specific toolchain attribute
   "__attribute__((section(".RamFunc")))".
*/
#define __RAM_FUNC HAL_StatusTypeDef  __attribute__((section(".RamFunc")))

#endif

#if !defined(__CLANG_ARM) && defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#define __CLANG_ARM
#endif

#ifndef HAL_SECTION
#if defined(__CC_ARM) || defined(__CLANG_ARM)           /* ARM Compiler */
#define HAL_SECTION(x)                  __attribute__((section(x)))
#elif defined (__IAR_SYSTEMS_ICC__)     /* for IAR Compiler */
#define HAL_SECTION(x)                  @ x
#elif defined (__GNUC__)                /* GNU GCC Compiler */
#define HAL_SECTION(x)                  __attribute__((section(x)))
#else
#define HAL_SECTION(x)
#endif
#endif

#define HAL_STRINGIFY_(val)       #val

/** Converts a macro argument into a character constant.
 */
#define HAL_STRINGIFY(val)       HAL_STRINGIFY_(val)

/** RAM non-retained code section */
#if  defined(_MSC_VER)
#define HAL_RAM_NON_RET_CODE_SECT(section_name, func)        func
#elif defined(__IAR_SYSTEMS_ICC__)
#define HAL_RAM_NON_RET_CODE_SECT(section_name, func)        func HAL_SECTION(HAL_STRINGIFY(.l1_non_ret_text_##section_name))
#else
#define HAL_RAM_NON_RET_CODE_SECT(section_name, func)        HAL_SECTION(HAL_STRINGIFY(.l1_non_ret_text_##section_name)) func
#endif /* _MSC_VER */


/** RAM non-retained rodata section */
#define HAL_RAM_NON_RET_RODATA_SECT(section_name)        HAL_SECTION(HAL_STRINGIFY(.l1_non_ret_rodata_##section_name))

/** RAM retained code section */
#if  defined(_MSC_VER)
#define HAL_RAM_RET_CODE_SECT(section_name, func)        func
#elif defined(__IAR_SYSTEMS_ICC__)
#define HAL_RAM_RET_CODE_SECT(section_name, func)        func HAL_SECTION(HAL_STRINGIFY(.l1_ret_text_##section_name))
#else
#define HAL_RAM_RET_CODE_SECT(section_name, func)        HAL_SECTION(HAL_STRINGIFY(.l1_ret_text_##section_name)) func
#endif /* _MSC_VER */

/** RAM retained rodata section */
#if  defined(_MSC_VER)
#define HAL_RAM_RET_RODATA_SECT(section_name, var)        var
#elif defined(__IAR_SYSTEMS_ICC__)
#define HAL_RAM_RET_RODATA_SECT(section_name, var)        var HAL_SECTION(HAL_STRINGIFY(.l1_ret_rodata_##section_name))
#else
#define HAL_RAM_RET_RODATA_SECT(section_name, var)        HAL_SECTION(HAL_STRINGIFY(.l1_ret_rodata_##section_name)) var
#endif /* _MSC_VER */

#if defined(__CC_ARM) || defined(__CLANG_ARM)
#define HAL_RETM_BSS_SECT(section_name, var)             var HAL_SECTION(HAL_STRINGIFY(.bss.retm_bss_##section_name))
#elif  defined(_MSC_VER)
#define HAL_RETM_BSS_SECT(section_name, var)             var
#else
#define HAL_RETM_BSS_SECT(section_name, var)             var HAL_SECTION(HAL_STRINGIFY(.bss.retm_bss_##section_name))
#endif /* __CC_ARM  */


/**
  * @brief  __NOINLINE definition
  */
#if defined ( __CC_ARM   ) || defined   (  __GNUC__  )
/* ARM & GNUCompiler
   ----------------
*/
#define __NOINLINE __attribute__ ( (noinline) )

#elif defined ( __ICCARM__ )
/* ICCARM Compiler
   ---------------
*/
#define __NOINLINE _Pragma("optimize = no_inline")

#endif


/**
  * @}
  */

/**
  * @}
  */



#ifdef __cplusplus
}
#endif

#endif /* ___BF0_HAL_DEF */
