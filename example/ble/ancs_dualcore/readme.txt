@使用指南

    @介绍：ancs_dualcore 工程是基于RTthreadOS，通过双核使用Sifli SDK ANCS service的示例。该工程可运行在SF32LB55X系列上。
                - 包含了BLE的广播、连接，以及对ANCS数据的基本处理。
                - BLE的service和application运行在LCPU(Low performance CPU)同时订阅ANCS的application运行在HCPU(High performance CPU)
    
    @Menuconfig配置：由于使用双核，该工程所用到BLE相关的配置会分为HCPU和LCPU：
                - Platform
                    - Select board peripherials->Select board: 选择运行平台，请根据实际Board选择LB55XXX系列。该选项需要在两个CPU对应选项中都选中并保证为同一个平台。
                - HCPU
                    - RTOS->On-chip Peripherial Drivers->Enable LCPU Patch[*]:
                    - RTOS->On-chip Peripherial Drivers->Enable LCPU image[*]: 打开LCPU Patch和image以便于LCPU正常运行。
                    - Sifli Middleware->Enable Data service[*]: 打开Data service，该service提供了一套数据提供者和数据订阅者的数据交互机制。
                        - Enable ble nvds service[*]: 打开Data service中的NVDS数据服务。该服务会提供NVDS的异步操作以便于运行在LCPU的BLE stack/service将数据存储到HCPU的flash。
                    - Third party packages->FlashDB[*]: 打开FlashDB提供访问Flash的接口，NVDS需要打开该服务
                - LCPU
                    - Sifli Middleware->Enable BLE service[*]： 打开BLE service，该service会提供BLE GAP/GATT/COMMON的服务
                        - Enable BLE ANCS[*]: 打开BLE ANCS service，该service会提供对IOS端ANCS服务的访问，获取Notification并提供给其用户。
                    - Sifli Middleware->Enable BLE stack[*]: 使能BLE协议栈。
                    - Sifli Middleware->Enable Data service[*]: 打开Data service，该service提供了一套数据提供者和数据订阅者的数据交互机制。ANCS使用该机制主要为了分离数据和UI。
                        - Enable ANCS service[*]: 打开Data service中的ANCS数据服务。该服务会配置及使能BLE ANCS service并对收到的通知进行处理，将处理后的数据提供给该服务的订阅者。

    @函数入口：
        - HCPU
            1. app_ancs_init(): 该函数会订阅Data service的ANCS服务，并注册callback获取订阅数据。
        - LCPU
            1. main(): 系统开始调度后会被call到，该函数会enable BLE service进而打开BLE，初始化OS的mailbox，并进入while loop。在收到蓝牙power on的通知后打开广播。
            2. ble_app_event_handler(): 该函数通过BLE_EVENT_REGISTER注册到BLE service中，处理GAP/GATT/Common等BLE相关的事件。
                    - 由于IOS要求配对以后才能访问ANCS，所以该handler里在连接后会主动发起配对。
            3. 自定义GATT service UUID：“00000000-0000-0070-7061-5F696C666973”。
                    - 自定义characteristic UUID: "00000000-0000-0170-7061-5F696C666973"

    @相关Shell命令
        - HCPU
            1. 出厂化BLE相关Flash数据：nvds reset_all 1
                    - 为避免Flash冲突，第一次使用最好先下该命令。
            2. 设置蓝牙MAC地址：nvds update addr 6 [addr]. Example: nvds update addr 6 2345670123C3

	@手机端建议：
		1. iPhone手机推荐用第三方软件LightBlue，Android端用nRF Connect进行BLE测试。

    @注意事项：
        - 该工程默认启用睡眠模式，一旦运行会很快进入睡眠模式。进入睡眠模式后，无法连接Jlink，Uart也无法输入，蓝牙可以正常工作。可以把PA80接地来强制不进入睡眠模式。
        - 需要先编译LCPU，再编译HCPU。HCPU会将LCPU的image整合到其code中。
        - 需要在LCPU的Kconfig里面“config LCPU_ROM”以及在rtconfig.py加上“CUSTOM_LFLAGS = 'rom.sym'”来启用LCPU ROM代码以减小RAM使用。