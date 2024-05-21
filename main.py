from hardware import *
from so import *
import log


##
##  MAIN 
##
if __name__ == '__main__':
    log.setupLogger()
    log.logger.info('Starting emulator')

    ## setup our hardware and set memory size to 25 "cells"
    HARDWARE.setup(22)

    ## new create the Operative System Kernel
    # "booteamos" el sistema operativo
    kernel = Kernel()

    prg1 = Program("prg1.exe", [ASM.CPU(2), ASM.IO(), ASM.CPU(3), ASM.IO(), ASM.CPU(2)])
    prg2 = Program("prg2.exe", [ASM.CPU(4), ASM.IO(), ASM.CPU(1)])
    prg3 = Program("prg3.exe", [ASM.CPU(3)])

    ## start
    HARDWARE.cpu.pc = 0
    HARDWARE.switchOn()

    # executamos los programas "concurrentemente"
    kernel.run(prg1)
    kernel.run(prg2)
    kernel.run(prg3)




