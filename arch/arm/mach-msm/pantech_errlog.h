#ifndef _SYS_LOGFILE_H_
#define _SYS_LOGFILE_H_

#define TRUE 1
#define FALSE 0
typedef unsigned char IDX_T;
typedef bool	    BOOL;
typedef uint32_t	UINT32;
typedef uint16_t  UINT16;
typedef uint8_t   UINT8;

extern const IDX_T NULL_IDX;


// 보관된 로그의 총수를 알려준다.
UINT8 pantech_logfile_get_log_count( void );

// 보관된 로그 메세지를 iteration하기 위한 함수
IDX_T pantech_logfile_get_first_log( char* log_buf );
IDX_T pantech_logfile_get_prev_log( IDX_T idx, char* log_buf );
IDX_T pantech_logfile_get_next_log( IDX_T idx, char* log_buf );
// iteration 중인 log위 위치를 알려준다.
UINT8 pantech_logfile_get_current_pos( void );

// 로그를 저장하는 함수
int pantech_logfile_save_log( const char* msg );

// 모든 로그를 삭제하는 함수
void pantech_errlog_remove_all_log( void );


void pantech_errlog_add_stack(const char *name, int size);

void pantech_errlog_add_log(const char *log, int size);

void pantech_errlog_init_log(void);

void pantech_errlog_save_log(void);

void pantech_errlog_noti_crash_to_marm(uint32_t reason);

void pantech_errlog_display_add_log(const char *log, int size);

void pantech_errlog_display(bool mArm, bool do_panic);

void pantech_errlog_display_with_errlog(bool mArm, bool do_panic);

#endif // _SYS_LOGFILE_H_

