#include "tactile_shm.h"

#ifdef USE_PTHREAD_MUTEX
int shm_lock_init(struct tactile_shm *shm)
{
  pthread_mutex_init(&(shm->cmd_lock), NULL);
  pthread_mutex_init(&(shm->info_lock), NULL);
}

int cmd_shm_lock(struct tactile_shm *shm)
{
  pthread_mutex_lock(&(shm->cmd_lock));
}

int info_shm_lock(struct tactile_shm *shm)
{
  pthread_mutex_lock(&(shm->info_lock));
}
int cmd_shm_unlock(struct tactile_shm *shm)
{
  pthread_mutex_unlock(&(shm->cmd_lock));
}
int info_shm_unlock(struct tactile_shm *shm)
{
  pthread_mutex_unlock(&(shm->info_lock));
}
#else
int cmpexchg(int *ptr, int old, int next)
{
  int res;
  __asm__ __volatile__("lock; cmpxchg %1, %2\n"
                       : "=a" (res)
                       : "q"(next), "m"(*ptr), "0"(old)
                       : "memory");
  return res;
}

void init_inter_system_lock(int *lock)
{
  *lock=0;
}

void inter_system_lock(int *lock)
{
  int res;
  do {
    res=cmpexchg(lock, 0, 1);
  }while(res != 0);
}

void inter_system_unlock(int *lock)
{
  *lock=0;
}

int shm_lock_init(struct tactile_shm *shm)
{
  init_inter_system_lock(&(shm->cmd_lock));
  init_inter_system_lock(&(shm->info_lock));
}

int cmd_shm_lock(struct tactile_shm *shm)
{
  inter_system_lock(&(shm->cmd_lock));
}

int info_shm_lock(struct tactile_shm *shm)
{
  inter_system_lock(&(shm->info_lock));
}

int cmd_shm_unlock(struct tactile_shm *shm)
{
  inter_system_unlock(&(shm->cmd_lock));
}

int info_shm_unlock(struct tactile_shm *shm)
{
  inter_system_unlock(&(shm->info_lock));
}

void *set_shared_memory(key_t _key, size_t _size)
{
  int shm_id;
  void *ptr;
  int err;
  size_t size = _size * 2;
  key_t key = _key;
  shm_id=shmget(key, size, 0666|IPC_CREAT);
  err = errno;
  if(shm_id==-1 && err == EINVAL) {
    size = _size;
    shm_id=shmget(key, size, 0666|IPC_CREAT);
    err = errno;
  }
  if(shm_id==-1) {
    fprintf(stderr, "shmget failed, key=%d, size=%d, errno=%d\n", key, size, err);
    return NULL;
  }
  ptr=(struct shared_data *)shmat(shm_id, (void *)0, 0);
  if(ptr==(void *)-1) {
    int err=errno;
    fprintf(stderr, "shmget failed, key=%d, size=%d, shm_id=%d, errno=%d\n", key, size, shm_id, err);
    return NULL;
  }
  return ptr;
}

#endif
