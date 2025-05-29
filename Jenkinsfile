pipeline {
   agent { label 'sdk' }
    stages {
        stage('Print CI Info') {
            steps {
                echo "[NODE_NAME]: ${env.NODE_NAME}\n[GERRIT_PROJECT]: ${env.GERRIT_PROJECT}\n[GERRIT_BRANCH]: ${env.GERRIT_BRANCH}\n[GERRIT_CHANGE_OWNER_EMAIL]: ${env.GERRIT_CHANGE_OWNER_EMAIL}\n[GERRIT_CHANGE_SUBJECT]: ${env.GERRIT_CHANGE_SUBJECT}\n[GIT_COMMIT]: ${env.GIT_COMMIT}" 
            }
        }
        stage('Parallel Stage') {
            //failFast true
            parallel {
                stage('common ec-lb583 hal') {
                    steps {
                        bat'''
                        tools\\autotest\\build.bat example\\hal_example\\project --board ec-lb583_hcpu
                        '''
                    }
                }
                stage('ec-lb587 watch') {
                   steps {
                       bat'''
                       tools\\autotest\\build.bat example\\multimedia\\lvgl\\watch\\project --board ec-lb587
                       '''
                   }
                }
                stage('common eh-lb563 bt') {
                    steps {
                        bat'''
                        tools\\autotest\\build.bat example\\bt\\test_example\\project --board eh-lb563_hcpu
                        '''
                    }
                }
                stage('common eh-lb523 rt_driver') {
                    steps {
                        bat'''
                        tools\\autotest\\build.bat example\\rt_driver\\project --board eh-lb523_hcpu
                        '''
                    }
                } 
                stage('common eh-lb551 ble') {
                    steps {
                        bat'''
                        tools\\autotest\\build.bat example\\ble\\central_and_peripheral\\project\\hcpu --board eh-lb551_hcpu
                        '''
                    }
                } 
            }
        }
         stage('Archive files') {
             steps {
                 //archiveArtifacts artifacts: 'example\boot_loader\project\butterflmicro\rom\build\bootloader.*', fingerprint: true
                 //archiveArtifacts artifacts: 'example\\boot_loader\\project\\butterflmicro\\rom\\build\\bootloader.*,example\\rom_bin\\lcpu_boot_loader\\lcpu_rom_micro.*,example\\rom_bin\\lcpu_boot_loader\\rom_micro.sym,example\\rom_bin\\lcpu_boot_loader\\rom_micro_internal.sym', fingerprint: true, onlyIfSuccessful: true
                 echo "Archive files success"
            }
         }
    }
}

