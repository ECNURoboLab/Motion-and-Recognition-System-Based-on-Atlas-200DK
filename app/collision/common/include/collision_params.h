#ifndef _COLLISION_H_
#define _COLLISION_H_

#include <sys/types.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/stat.h>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>


#define MAX_OBJECT_NUM 10

#pragma pack(1)
typedef struct object_bbox {
  int x;
  int y;
  int width;
  int height;
  float confidence;
}S_OBJECT_BBOX;
#pragma pack()

#pragma pack(1)
typedef struct env_status {
  int init_param;
  int env_block;
  int obj_number;
  S_OBJECT_BBOX object[MAX_OBJECT_NUM];
}S_ENV_STATUS;
#pragma pack()

class ShmServer {
public:
  ShmServer(){
    init_status = false;
    shm_size    = sizeof(struct env_status);
    // Init
    if (ShmInit() < 0) {
      printf("SHM GET ID ERROR\n");
    }
    // Get Addr
    if (ShmGetAddr() == NULL) {
      printf("SHM GET ADDR ERROR\n");
    }
    // Name struct
    p_env_status = (struct env_status *)shm_addr;
    if (p_env_status == NULL) {
      printf("SHM ENABLE ERROR\n");
    }
    else{
      p_env_status->init_param = 0x01;
      init_status = true;
    }
  }

  int ShmInit() {
    shm_fd = shmget(123559, shm_size, IPC_CREAT | 0777);
    return shm_fd;
  }

  void* ShmGetAddr() {
    shm_addr = shmat(shm_fd, NULL, 0);
    return shm_addr;
  }

  ~ShmServer() {
    init_status = false;

    int shmdt_ret = shmdt(shm_addr);
    if (shmdt_ret < 0) {
      printf("SHM DETACHE FAILED\n");
    }

    int mctl_ret = shmctl(shm_fd, IPC_RMID, NULL);
    if ( mctl_ret < 0) {
      printf("SHM REMOVE MEMORY FAILED\n");
    }
  }

  env_status *GetEnvStatus() {
    return p_env_status;
  }

private:
  bool    init_status;
  int     shm_fd;
  void    *shm_addr;
  int     shm_size;
  S_ENV_STATUS *p_env_status;
};


#endif