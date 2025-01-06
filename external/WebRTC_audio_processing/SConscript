import os
from building import *
import rtconfig

src = []
inc = []


# get current directory

cwd = GetCurrentDir()
inc    += [cwd]
inc    += [cwd + "/webrtc/common_audio"]
inc    += [cwd + "/webrtc/common_audio/vad/include"]
inc    += [cwd + "/webrtc/common_audio/signal_processing"]
inc    += [cwd + "/webrtc/common_audio/signal_processing/include"]
inc    += [cwd + "/webrtc/modules/audio_processing/ns"]
inc    += [cwd + "/webrtc/modules/audio_processing/ns/include"]
inc    += [cwd + "/webrtc/system_wrappers/include"]

common_src  = Glob('./webrtc/common_audio/*.c')
common_src += Glob('./webrtc/common_audio/vad/*.c')
common_src += Glob('./webrtc/common_audio/signal_processing/*.c')

src += common_src

if GetDepend('WEBRTC_ANS_FLOAT'):
    ns_src = Glob('./webrtc/modules/audio_processing/ns/noise_suppression.c')
    ns_src += Glob('./webrtc/modules/audio_processing/ns/ns_core.c')
    src += ns_src

if GetDepend('WEBRTC_ANS_FIX'):
    ns_src = Glob('./webrtc/modules/audio_processing/ns/noise_suppression_x.c')
    ns_src += Glob('./webrtc/modules/audio_processing/ns/nsx_core.c')
    ns_src += Glob('./webrtc/modules/audio_processing/ns/nsx_core_c.c')
    src += ns_src

if GetDepend('WEBRTC_AGC_FIX'):
    agc_src = Glob('./webrtc/modules/audio_processing/agc/legacy/*.c')
    inc    += [cwd + "/webrtc/modules/audio_processing/agc/legacy"]
    src += agc_src

if GetDepend('WEBRTC_AEC'):
    aec_src = Glob('./webrtc/modules/audio_processing/aec/*.c')
    aec_src += Glob('./webrtc/modules/audio_processing/utility/*.c')
    src += aec_src
    inc    += [cwd + "/webrtc/modules/audio_processing/aec"]
    inc    += [cwd + "/webrtc/modules/audio_processing/aec/include"]
    inc    += [cwd + "/webrtc/modules/audio_processing/logging"]
    inc    += [cwd + "/webrtc/modules/audio_processing/utility"]

if GetDepend('WEBRTC_AECM'):
    aec_src = Glob('./webrtc/modules/audio_processing/aecm/*.c')
    aec_src += Glob('./webrtc/modules/audio_processing/utility/*.c')
    src += aec_src
    inc    += [cwd + "/webrtc/modules/audio_processing/aecm"]
    inc    += [cwd + "/webrtc/modules/audio_processing/aecm/include"]
    inc    += [cwd + "/webrtc/modules/audio_processing/logging"]
    inc    += [cwd + "/webrtc/modules/audio_processing/utility"]

src += Glob('./webrtc/modules/audio_processing/dc_correction/*.c')
inc    += [cwd + "/webrtc/modules/audio_processing/dc_correction"]
src += Glob('./webrtc/modules/audio_processing/ramp_in/*.c')
inc    += [cwd + "/webrtc/modules/audio_processing/ramp_in"]
src += Glob('./webrtc/modules/audio_processing/ramp_out/*.c')
inc    += [cwd + "/webrtc/modules/audio_processing/ramp_out"]

LOCAL_CCFLAGS = ''


#demo
if GetDepend('WEBRTC_ANS_DEMO'):
    src += ['./webrtc_ns_test.c']

if GetDepend('WEBRTC_AGC_DEMO'):
    src += ['./webrtc_agc_test.c']

if GetDepend('WEBRTC_AEC_DEMO'):
    src += ['./webrtc_aec_test.c']

src += ['./webrtc_vad_test.c']

#group = DefineGroup('ffmpeg', src, depend = ['PKG_USING_FFMPEG'], CPPPATH = inc, LOCAL_CCFLAGS = LOCAL_CCFLAGS)
group = DefineGroup('webrtc', src, depend = ['PKG_USING_WEBRTC'], CPPPATH = inc, LOCAL_CCFLAGS = LOCAL_CCFLAGS)

Return('group')
