#ifdef MELODY_NK
/***************************************************
 *
 * Module : MELODY
 * Description : melody types
 *
 ***************************************************/
typedef enum MELODY_DATA {
  MELODY_SUCCESS = 0,
  MELODY_READY = 1,
  MELODY_FAILS = 2
} eMELODY_DATA_t;


extern void MUSIC_Play(int mode);

#endif  /* MELODY_NK */
