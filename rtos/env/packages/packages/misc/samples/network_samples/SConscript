from building import *

src   = []
cwd   = GetCurrentDir()
include_path = [cwd]

# add network samples.
if GetDepend('NETWORK_SAMPLES_USING_TCP_CLIENT'):
    src += ['tcpclient_sample.c']

if GetDepend('NETWORK_SAMPLES_USING_TCP_SERVER'):
    src += ['tcpserver_sample.c']

if GetDepend('NETWORK_SAMPLES_USING_UDP_CLIENT'):
    src += ['udpclient_sample.c']

if GetDepend('NETWORK_SAMPLES_USING_UDP_SERVER'):
    src += ['udpserver_sample.c']

if GetDepend('NETWORK_SAMPLES_USING_TCP_CLIENT_SELECT'):
    src += ['tcpclient_select_sample.c']

if GetDepend('NETWORK_SAMPLES_USING_HTTP_CLIENT'):
    src += ['httpclient_sample.c']

group = DefineGroup('network-samples', src, depend = ['PKG_USING_NETWORK_SAMPLES'], CPPPATH = include_path)

Return('group')
