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


// ������ �α��� �Ѽ��� �˷��ش�.
UINT8 pantech_logfile_get_log_count( void );

// ������ �α� �޼����� iteration�ϱ� ���� �Լ�
IDX_T pantech_logfile_get_first_log( char* log_buf );
IDX_T pantech_logfile_get_prev_log( IDX_T idx, char* log_buf );
IDX_T pantech_logfile_get_next_log( IDX_T idx, char* log_buf );
// iteration ���� log�� ��ġ�� �˷��ش�.
UINT8 pantech_logfile_get_current_pos( void );

// �α׸� �����ϴ� �Լ�
int pantech_logfile_save_log( const char* msg );

// ��� �α׸� �����ϴ� �Լ�
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

