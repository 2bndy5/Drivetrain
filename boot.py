# This file is executed on every boot (including wake-boot from deepsleep)
#import esp
#esp.osdebug(None)
import uos, machine
#uos.dupterm(None, 1) # disable REPL on UART(0)
import gc
import webrepl
import network
sta_if = network.WLAN(network.STA_IF)
sta_if.active(True)
sta_if.connect('Casa_Bellwood', 'fu11H0U5e')
webrepl.start()
gc.collect()
