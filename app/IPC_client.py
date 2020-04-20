import struct
import time
from ctypes import *

# Param
shm_size = 32
shm_key = 123559
shm_id = -1
shm_addr = -1


class ShmClient():
  def __init__(self):
    self.SHM_SIZE = shm_size
    self.SHM_KEY = shm_key

    # Init C lib
    try:
      rt = CDLL('librt.so') 
    except:
      rt = CDLL('librt.so.1')

    # Declare shm
    # Get the shared memory
    # Param: (key, size, shmflg)
    self.shmget = rt.shmget
    self.shmget.argtypes = [c_int, c_size_t, c_int]
    self.shmget.restype = c_int

    # Attach the shared memory by shmid
    # Param: (shmid, shmaddr, shmflg)
    self.shmat = rt.shmat
    self.shmat.argtypes = [c_int, POINTER(c_void_p), c_int]
    self.shmat.restype = c_void_p

    # Detatch the shared memory
    # Param: shmaddr
    shmdt = rt.shmdt
    shmdt.argtypes = [POINTER(c_void_p)]
    shmdt.restype = c_int


  def callback_print(self):
    global shm_id
    global shm_addr
    if shm_id < 0:
      shm_id = self.shmget(self.SHM_KEY, self.SHM_SIZE, 0o777)
      if shm_id >= 0:
        shm_addr = self.shmat(shm_id, None, 0)

    target = []
    env_block = struct.unpack('i', string_at(shm_addr+1*4, 4))[0]
    obj_number = struct.unpack('i', string_at(shm_addr+2*4, 4))[0]
    for _ in range(0, obj_number):
      target_object = struct.unpack('iiiif', string_at(shm_addr+3*4, 20))  
      target.append(target_object)

    print("--------------------------------------------")
    print("Block Status: ", env_block)
    print("Object Number: ", obj_number)
    for i in range(0, obj_number):
      print("Object", i, ": ", target[i])
    

if __name__ == "__main__":
  client = ShmClient()
  while True:
    client.callback_print()
    time.sleep(1)