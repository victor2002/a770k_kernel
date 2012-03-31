#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>

#include <linux/string.h>
#include <linux/time.h>
#include <linux/fb.h>
#include <linux/kd.h>
#include <linux/syscalls.h>

#ifdef CONFIG_SW_RESET
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/nmi.h>
#include <mach/msm_iomap.h>
#endif

#include "pantech_errlog.h"
#include "pantech_fileio.h"
#include "sky_sys_reset.h"

#define FS_NULL_HANDLE	NULL
#define CLK_OFFSET_S	432000 /* 1 Jan 1980 -> 6 Jan 1980 */
#define QUAD_YEAR (366+(3*365))

typedef struct file *fs_handle_type;

#define LOG_DIR	"/data/errlog"
static const char   LOG_PATH[] = "/data/errlog/error.log";
#define LOG_SIZE  512// 각 로그의 크기는 512bytes
#define MAX_LOG   150// 저장할 로그의 수는 255를 초과하면 안 됨
#define  NULL_IDX 0xFF

/* For Display */
#define DISPLAY_LOG_SIZE	1024
static char *pDisplayBuffer;
static int DisplayLogIdx;
/* For Error Log */
static char *pErrorLog;
static int LogIdx;

typedef struct {
    BOOL   inited_;
    UINT32 first_log_pos_;
    UINT8  count_;
    UINT8  pos_;
    IDX_T  head_;
    IDX_T  tail_;
    IDX_T  free_head_;
    IDX_T  prev_[ MAX_LOG ];
    IDX_T  next_[ MAX_LOG ];
} LOG_HEADER_T;

typedef struct tag_TMR_DATE_T {
	unsigned short year;
	unsigned short month;
	unsigned short day;
	unsigned short hour;
	unsigned short minute;
	unsigned short second;
} TMR_DATE_T, clk_julian_type;

/*
================================================================================    : 82 bytes
                                 로그 파일 헤더                                     : 49
--------------------------------------------------------------------------------    : 82
  - 로그크기  : 0x0200                                                              : 24
  - MAX       : 0x32                                                                : 22
  - FIRST LOG : 0x03EA                                                              : 24
  - 로그수    : 0x03                                                                : 22
  - head      : 0x00                                                                : 22
  - tail      : 0x02                                                                : 22
  - free head : 0x03                                                                : 22
  - prev/next : FF01 0002 01FF FF04 FF05 FF06 FF07 FF08 FF09 FF0A FF0B FF0C         : 77
                FF0D FF0E FF0F FF10 FF11 FF12 FF13 FF14 FF15 FF16 FF17 FF18         : 77
                FF19 FF1A FF1B FF1C FF1D FF1E FF1F FF20 FF21 FF22 FF23 FF24         : 77
                FF25 FF26 FF27 FF28 FF29 FF2A FF2B FF2C FF2D FF2E FF2F FF30         : 77
                FF31 FFFF                                                           : 27
================================================================================    : 82
                                                                                    : 2
                                                                                    : 2
--------------------------------------------------------------------------------    : 82
                                    로그 001                                        : 46
--------------------------------------------------------------------------------    : 82
LOG_SIZE 만큼의 로그 저장                                                           : LOG_SIZE + 2
                                                                                    : 2
--------------------------------------------------------------------------------
                                    로그 002
--------------------------------------------------------------------------------
LOG_SIZE 만큼의 로그 저장

                           .    .    .    .    .    .
                           .    .    .    .    .    .
*/

static const UINT32 LOG_SIZE_POS  = 229; // 0x0200 위치, 82 + 49 + 82 + 16
static const UINT32 MAX_POS       = 253; // 0x32 위치,   LOG_SIZE_POS + 24
static const UINT32 FIRST_LOG_POS = 275; // 0x0000 위치, MAX_POS + 22
static const UINT32 LOG_COUNT_POS = 299; // 0x03 위치,   FIRST_LOG_POS + 22
static const UINT32 HEAD_POS      = 321; // 0x00 위치,   LOG_COUNT_POS + 22
static const UINT32 TAIL_POS      = 343; // 0x02 위치,   HEAD_POS + 22
static const UINT32 FREE_HEAD_POS = 365; // 0x03 위치,   TAIL_POS + 22
static const UINT32 PREV_NEXT_POS = 387; // FF01 위치,   FREE_HEAD_POS + 22

static const char TITLE_LINE[]     = "================================================================================\r\n";
static const char TITLE_MSG[]      = "                                 로그 파일 헤더\r\n";
static const char TITLE_SUB_LINE[] = "--------------------------------------------------------------------------------\r\n";


static void init_log( void );
static fs_handle_type open_log_file( void );
static BOOL create_log_file( void );
static BOOL load_head( LOG_HEADER_T* head );
static BOOL check_head_valid( void );
static UINT32 get_value( fs_handle_type file, UINT32 pos, UINT32 size );
static BOOL set_value( fs_handle_type file, UINT32 pos, UINT32 value, UINT32 size );
static BOOL get_prev_next( fs_handle_type file, LOG_HEADER_T* head );
static BOOL set_prev_next( fs_handle_type file, LOG_HEADER_T* head, UINT32 idx );
static UINT32 hex_to_dec( const char* hex, UINT32 len );

static IDX_T add_log( fs_handle_type file, LOG_HEADER_T* head, const char* msg );
static BOOL get_log( IDX_T idx, char* log_buf );


/*
 * The year_tab table is used for determining the number of days which
 * have elapsed since the start of each year of a leap year set. It has
 * 1 extra entry which is used when trying to find a 'bracketing' year.
 * The search is for a day >= year_tab[i] and day < year_tab[i+1].
 */
static const UINT16 year_tab[] = {
  0,                              /* Year 0 (leap year) */
  366,                            /* Year 1             */
  366+365,                        /* Year 2             */
  366+365+365,                    /* Year 3             */
  366+365+365+365                 /* Bracket year       */
};

/*
 * The norm_month_tab table holds the number of cumulative days that have
 * elapsed as of the end of each month during a non-leap year.
 */
static const UINT16 norm_month_tab[] = {
  0,                                    /* --- */
  31,                                   /* Jan */
  31+28,                                /* Feb */
  31+28+31,                             /* Mar */
  31+28+31+30,                          /* Apr */
  31+28+31+30+31,                       /* May */
  31+28+31+30+31+30,                    /* Jun */
  31+28+31+30+31+30+31,                 /* Jul */
  31+28+31+30+31+30+31+31,              /* Aug */
  31+28+31+30+31+30+31+31+30,           /* Sep */
  31+28+31+30+31+30+31+31+30+31,        /* Oct */
  31+28+31+30+31+30+31+31+30+31+30,     /* Nov */
  31+28+31+30+31+30+31+31+30+31+30+31   /* Dec */
};

/*
 * The leap_month_tab table holds the number of cumulative days that have
 * elapsed as of the end of each month during a leap year.
 */
static const UINT16 leap_month_tab[] = {
  0,                                    /* --- */
  31,                                   /* Jan */
  31+29,                                /* Feb */
  31+29+31,                             /* Mar */
  31+29+31+30,                          /* Apr */
  31+29+31+30+31,                       /* May */
  31+29+31+30+31+30,                    /* Jun */
  31+29+31+30+31+30+31,                 /* Jul */
  31+29+31+30+31+30+31+31,              /* Aug */
  31+29+31+30+31+30+31+31+30,           /* Sep */
  31+29+31+30+31+30+31+31+30+31,        /* Oct */
  31+29+31+30+31+30+31+31+30+31+30,     /* Nov */
  31+29+31+30+31+30+31+31+30+31+30+31   /* Dec */
};

/*
 * The day_offset table holds the number of days to offset as of the end of
 * each year.
 */
static const UINT16 day_offset[] = {
  1,                                    /* Year 0 (leap year) */
  1+2,                                  /* Year 1             */
  1+2+1,                                /* Year 2             */
  1+2+1+1                               /* Year 3             */
};

// paiksun...
#ifdef CONFIG_SW_RESET
extern int pm8058_get_pwrkey_status(void);
#endif

static UINT32 tmr_GetLocalTime(void)
{
    struct timeval tv;
    UINT32 seconds;

    do_gettimeofday(&tv);
    seconds = (UINT32)tv.tv_sec;
    /* Offset to sync timestamps between Arm9 & Arm11
     Number of seconds between Jan 1, 1970 & Jan 6, 1980 */
    seconds = seconds - (10*365+5+2)*24*60*60 ;
    return seconds; // after Jan 1, 1970.
}

static UINT32 div4x2( UINT32 dividend, unsigned short divisor, unsigned short *rem_ptr)
{
	*rem_ptr = (unsigned short) (dividend % divisor);
	return (dividend /divisor);
}

static void clk_secs_to_julian(UINT32 secs, clk_julian_type *julian_ptr)
{
	int i;
	unsigned short days;

	//secs += CLK_OFFSET_S;
	secs = div4x2( secs, 60, &julian_ptr->second );
	secs = div4x2( secs, 60, &julian_ptr->minute );
	secs = div4x2( secs, 24, &julian_ptr->hour );
	julian_ptr->year = 1980 + ( 4 * div4x2( secs, QUAD_YEAR, &days));

	for(i = 0; days >= year_tab[i + 1]; i++);
	days -= year_tab[i];
	julian_ptr->year += i;

	if(i == 0)
	{
		for(i = 0; days >= leap_month_tab[i + 1]; i++ );
		julian_ptr->day = days - leap_month_tab[i] + 1;
		julian_ptr->month = i + 1;

	} else {
		for(i = 0; days >= norm_month_tab[i + 1]; i++ );
		julian_ptr->day = days - norm_month_tab[i] + 1;
		julian_ptr->month = i + 1;
	}
}

static void tmr_GetDate( UINT32 dseconds, TMR_DATE_T *pDate)
{
	if(pDate) {
		if(dseconds == 0) {
			memset(pDate, 0x00, sizeof(TMR_DATE_T));
			pDate->year = 1980;
			pDate->month = 1;
			pDate->day = 6;
		} else {
			clk_secs_to_julian(dseconds, (clk_julian_type *)pDate);
		}
	}
}

static const char* get_build_version( void )
{
    return FIRM_VER;
}


static LOG_HEADER_T g_head;
static char g_blank_buf[ LOG_SIZE + 2 ];


UINT8 pantech_logfile_get_log_count( void )
{
    init_log();
    return g_head.count_;
}

IDX_T pantech_logfile_get_first_log( char* log_buf )
{
    init_log();
    g_head.pos_ = 0;

    if( g_head.head_ == NULL_IDX )
    {
        return NULL_IDX;
    }

    if( get_log( g_head.head_, log_buf ) == FALSE )
    {
        return NULL_IDX;
    }

    return g_head.head_;
}


IDX_T pantech_logfile_get_prev_log( IDX_T idx, char* log_buf )
{
    if( g_head.inited_ == FALSE || idx == NULL_IDX )
    {
        return NULL_IDX;
    }

    if( g_head.prev_[ idx ] == NULL_IDX )
    {
        idx         = g_head.tail_;
        g_head.pos_ = g_head.count_ - 1;
    }
    else
    {
        idx = g_head.prev_[ idx ];
        g_head.pos_--;
    }

    if( get_log( idx, log_buf ) == FALSE )
    {
        return NULL_IDX;
    }

    return idx;
}


IDX_T pantech_logfile_get_next_log( IDX_T idx, char* log_buf )
{
    if( g_head.inited_ == FALSE || idx == NULL_IDX )
    {
        return NULL_IDX;
    }

    if( g_head.next_[ idx ] == NULL_IDX )
    {
        idx         = g_head.head_;
        g_head.pos_ = 0;
    }
    else
    {
        idx = g_head.next_[ idx ];
        g_head.pos_++;
    }

    if( get_log( idx, log_buf ) == FALSE )
    {
        return NULL_IDX;
    }

    return idx;
}


UINT8 pantech_logfile_get_current_pos( void )
{
    return g_head.pos_;
}


int pantech_logfile_save_log( const char* msg )
{
    fs_handle_type  file;

    file = open_log_file();
    if( file == FS_NULL_HANDLE )
    {
        return -1;
    }

    if( g_head.count_ < MAX_LOG )
    {
        IDX_T new_idx;

        new_idx = add_log( file, &g_head, msg );
        if( new_idx == NULL_IDX ) return FALSE;

        if( set_value( file, LOG_COUNT_POS, g_head.count_, 4 )     == FALSE )      return FALSE;
        if( set_value( file, TAIL_POS, g_head.tail_, 4 )           == FALSE )      return FALSE;
        if( set_value( file, FREE_HEAD_POS, g_head.free_head_, 4 ) == FALSE )      return FALSE;
        if( set_prev_next( file, &g_head, new_idx )                == FALSE )      return FALSE;

        if( g_head.count_ == 1 )
        {
            if( set_value( file, HEAD_POS, g_head.head_, 4 )       == FALSE )      return FALSE;
        }
        else
        {
            if( set_prev_next( file, &g_head, g_head.prev_[ new_idx ] ) == FALSE ) return FALSE;
        }
    }
    else
    {
        IDX_T idx;

        idx                 = g_head.head_;
        g_head.head_        = g_head.next_[ idx ];

        g_head.free_head_   = idx;
        g_head.next_[ idx ] = NULL_IDX;
        g_head.count_--;

        if( add_log( file, &g_head, msg ) != idx ) return FALSE;

        if( set_value( file, HEAD_POS, g_head.head_, 4 )        == FALSE ) return FALSE;
        if( set_value( file, TAIL_POS, g_head.tail_, 4 )        == FALSE ) return FALSE;
        if( set_prev_next( file, &g_head, idx )                 == FALSE ) return FALSE;
        if( set_prev_next( file, &g_head, g_head.prev_[ idx ] ) == FALSE ) return FALSE;
    }

    pantech_fclose( file );

    return 0;
}


void pantech_logfile_remove_all_log( void )
{
    pantech_fremove( LOG_PATH );
    
    memset( &g_head, 0, sizeof( g_head ) );
}


static void init_log( void )
{
    if( g_head.inited_ == FALSE )
    {
        g_head.inited_ = TRUE;

        memset( g_blank_buf, ' ', LOG_SIZE );
        g_blank_buf[ LOG_SIZE ]     = '\r';
        g_blank_buf[ LOG_SIZE + 1 ] = '\n';

        load_head( &g_head );
    }
}


static fs_handle_type open_log_file( void )
{
    fs_handle_type file;
    init_log();

    file = pantech_fopen( LOG_PATH, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR );
		if(file == NULL) {
			printk("Cant open file\n");
			return NULL;
		}
		
		return file;

}


static BOOL create_log_file( void )
{
    fs_handle_type      file;
    char                buf[ 128 ];
    UINT32              size;
    int                 row, col, count;
    int                 a;
    UINT32              first_log_pos = 0;

    if((file = pantech_fopen( LOG_PATH, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR)) == NULL) {
    	printk("Cant open file\n");
    	return false;
		}

    pantech_fwrite( file, (char*)TITLE_LINE, sizeof( TITLE_LINE ) - 1);
    pantech_fwrite( file, (char*)TITLE_MSG, sizeof( TITLE_MSG ) - 1);
    pantech_fwrite( file, (char*)TITLE_SUB_LINE, sizeof( TITLE_SUB_LINE ) - 1);

    sprintf( buf, "  - 로그크기  : 0x%04X\r\n", (int)LOG_SIZE );
    size = strlen( buf );
    pantech_fwrite( file, buf, size);

    sprintf( buf, "  - MAX       : 0x%02X\r\n", (int)MAX_LOG );
    size = strlen( buf );
    pantech_fwrite( file, buf, size);

    sprintf( buf, "  - FIRST LOG : 0x0000\r\n" );
    size = strlen( buf );
    pantech_fwrite( file, buf, size);

    sprintf( buf, "  - 로그수    : 0x00\r\n" );
    size = strlen( buf );
    pantech_fwrite( file, buf, size);

    sprintf( buf, "  - head      : 0xFF\r\n" );
    size = strlen( buf );
    pantech_fwrite( file, buf, size);

    sprintf( buf, "  - tail      : 0xFF\r\n" );
    size = strlen( buf );
    pantech_fwrite( file, buf, size);

    sprintf( buf, "  - free head : 0x00\r\n" );
    size = strlen( buf );
    pantech_fwrite( file, buf, size );

    buf[ 0 ] = ' ';
    buf[ 1 ] = 'F';
    buf[ 2 ] = 'F';
    for( row = 0, count = 0; TRUE; ++row )
    {
        char* str;

        if( row == 0 )
        { //       123456789012345
            str = "  - prev/next :";
        }
        else
        { //       123456789012345
            str = "               ";
        }
        pantech_fwrite( file, str, 15);

        for( col = 0; col < 12; ++col )
        {
            if( ++count == MAX_LOG )
            {
                pantech_fwrite( file, " FFFF", 5);
            }
            else
            {
                sprintf( buf + 3, "%02X", count );
                pantech_fwrite( file, buf, 5);
            }

            if( count == MAX_LOG )
            {
                pantech_fwrite( file, "\r\n", 2);
                goto _loop_end;
            }
            if( col == 11 )
            {
                pantech_fwrite( file, "\r\n", 2);
            }
        }
    }
_loop_end:

    pantech_fwrite( file, (char*)TITLE_LINE, sizeof( TITLE_LINE ) - 1);
    pantech_fwrite( file, "\r\n", 2);

    for( a = 0; a < MAX_LOG; ++a )
    {
        sprintf( buf, "                                    로그 %03d\r\n", a + 1 );

        pantech_fwrite( file, "\r\n", 2 );
        pantech_fwrite( file, (char*)TITLE_SUB_LINE, sizeof( TITLE_SUB_LINE ) - 1);
        pantech_fwrite( file, buf, strlen( buf ));
        pantech_fwrite( file, (char*)TITLE_SUB_LINE, sizeof( TITLE_SUB_LINE ) - 1);

        if( a == 0 )
        {
            first_log_pos = pantech_ftell(file);
        }
        pantech_fwrite( file, g_blank_buf, LOG_SIZE + 2);
    }

    sprintf( buf, "%04X", (int)first_log_pos );
    pantech_fseek( file, SEEK_SET, FIRST_LOG_POS + 2 );
    pantech_fwrite( file, buf, 4);
    pantech_fclose( file );

    return TRUE;
}


static BOOL load_head( LOG_HEADER_T* head )
{
    BOOL            create = FALSE;
    fs_handle_type  file;

		sys_mkdir(LOG_DIR, 0700);//make directory..

    if((file = pantech_fopen( LOG_PATH, O_RDONLY, S_IRUSR | S_IWUSR)) != NULL) {
    	if( check_head_valid() == FALSE )
			{
				create = TRUE;
			}
			pantech_fclose(file);
		} else {
			create = TRUE;
		}

    if( create == TRUE )
    {
        if( create_log_file() == FALSE )
        {
            return FALSE;
        }
    }

    file = open_log_file();
    if( file == FS_NULL_HANDLE )
    {
        return FALSE;
    }

    head->first_log_pos_ = get_value( file, FIRST_LOG_POS, 6 ); // FIRST LOG
    head->count_         = get_value( file, LOG_COUNT_POS, 4 ); // 로그수
    head->head_          = get_value( file, HEAD_POS, 4 );      // head
    head->tail_          = get_value( file, TAIL_POS, 4 );      // tail
    head->free_head_     = get_value( file, FREE_HEAD_POS, 4 ); // free head
    if( head->first_log_pos_ == (UINT32)-1 ||
        head->count_         == (UINT32)-1 ||
        head->head_          == (UINT32)-1 ||
        head->tail_          == (UINT32)-1 ||
        head->free_head_     == (UINT32)-1 )
    {
        goto _fail;
    }

    if( get_prev_next( file, head ) == FALSE )
    {
        goto _fail;
    }

    pantech_fclose( file );
    return TRUE;

_fail:
    pantech_fclose( file );
    return FALSE;
}


static BOOL check_head_valid( void )
{
    fs_handle_type  file;

    file = open_log_file();
    if( file == FS_NULL_HANDLE )
    {
        return FALSE;
    }

    // 로그크기(LOG_SIZE) 동일한지 검사
    if( get_value( file, LOG_SIZE_POS, 6 ) != LOG_SIZE )
    {
        goto _fail;
    }

    // 로그최대개수(MAX_LOG) 일치하는지 검사
    if( get_value( file, MAX_POS, 4 ) != MAX_LOG )
    {
        goto _fail;
    }

    pantech_fclose( file );
    return TRUE;

_fail:
    pantech_fclose( file );
    pantech_fremove( LOG_PATH );
    return FALSE;
}


static UINT32 get_value( fs_handle_type file, UINT32 pos, UINT32 size )
{
    char buf[ 16 ];
    int res;

    pantech_fseek( file, SEEK_SET, pos );
    res = pantech_fread( file, buf, size );

    if( res != size )
    {
        return (UINT32)-1;
    }

    return hex_to_dec( buf + 2, size - 2 );
}


static BOOL set_value( fs_handle_type file, UINT32 pos, UINT32 value, UINT32 size )
{
    char buf[ 16 ];
    int res;

    if( size == 4 )
    {
        sprintf( buf, "0x%02X", (int)value );
    }
    else if( size == 6 )
    {
        sprintf( buf, "0x%04X", (int)value );
    }
    else
    {
        return FALSE;
    }

    pantech_fseek( file, SEEK_SET, pos);
    res = pantech_fwrite( file, buf, size );
    if( res != size )
    {
        return FALSE;
    }

    return TRUE;
}


static BOOL get_prev_next( fs_handle_type file, LOG_HEADER_T* head )
{
    int             row, col, count;
    UINT32          pos;
    char            buf[ 4 ];
    int             res;

    pos = PREV_NEXT_POS;
    for( row = 0, count = 0; TRUE; ++row )
    {
        for( col = 0; col < 12; ++col )
        {
						pantech_fseek( file, SEEK_SET, pos );
						res = pantech_fread( file, buf, 4 );

            if( res != 4 )
            {
                return FALSE;
            }
            head->prev_[ count ] = hex_to_dec( buf, 2 );
            head->next_[ count ] = hex_to_dec( buf + 2, 2 );

            if( ++count == MAX_LOG )
            {
                return TRUE;
            }
            if( col == 11 )
            {
                pos += 4 + 2 + 16; // 현재 prev/next 4, "\r\n" 2, 공백 16
            }
            else
            {
                pos += 4 + 1; // FF01과 공백
            }
        }
    }

    // 이 곳까지 올 수 없다.
    printk("get_prev_next ASSERT\n");

    return FALSE;
}


static BOOL set_prev_next( fs_handle_type file, LOG_HEADER_T* head, UINT32 idx )
{
    UINT32          pos;
    char            buf[ 5 ];
    UINT32          nSize;
    int             res;

    pos = PREV_NEXT_POS
          + (UINT32)( idx / 12 ) * ( 16 + ( 4 + 1 ) * 11 + 4 + 2 )
          + ( idx % 12 ) * ( 4 + 1 );
    nSize = sprintf( buf, "%02X%02X", head->prev_[ idx ], head->next_[ idx ] );

    if(nSize >= 5 )
    	return FALSE;

    pantech_fseek( file, SEEK_SET, pos);
    res = pantech_fwrite( file, buf, 4 );

    if( res != 4 )
    {
        return FALSE;
    }

    return TRUE;
}


static UINT32 hex_to_dec( const char* hex, UINT32 len )
{
    UINT32 decimal = 0;
    UINT32 order   = 1;
    int    i       = 0;
    char   ch      = 0;

    for( i = len - 1; i >= 0; --i )
    {
        ch = hex[ i ];

        if( ch >= '0' && ch <= '9' )
        {
            decimal += order * ( ch - '0' );
        }
        else if( ch >= 'A' && ch <= 'F' )
        {
            decimal += order * ( ch - 'A' + 10 );
        }
        else if( ch >= 'a' && ch <= 'f' )
        {
            decimal += order * ( ch - 'a' + 10 );
        }
        else
        {
            return (UINT32)-1;
        }

        order *= 16;
    }

    return decimal;
}


static IDX_T add_log( fs_handle_type file, LOG_HEADER_T* head, const char* msg )
{
    UINT32          idx = head->free_head_;
    UINT32          pos, msg_size, ver_size, size;
    int             res;

    enum { DATE_SIZE = 16 }; // "20080810 113000 " >> 20bytes
    TMR_DATE_T  date;
    char            date_buf[ DATE_SIZE + 1 ];

    if( idx == NULL_IDX )
    {
        return NULL_IDX;
    }
    head->free_head_ = head->next_[ idx ];

    if( head->count_ == 0 )
    {
        head->head_ = idx;
    }
    else
    {
        head->next_[ head->tail_ ] = idx;
    }
    head->count_++;

    head->prev_[ idx ] = head->tail_;
    head->next_[ idx ] = NULL_IDX;
    head->tail_ = idx;

    pos = head->first_log_pos_ + idx * ( 82 + 46 + 82 + LOG_SIZE + 2 + 2 );

    // 현재 시각 정보
    size = DATE_SIZE;
    tmr_GetDate( tmr_GetLocalTime(), &date );
    msg_size = sprintf( date_buf, "%4d%02d%02d %02d%02d%02d ", date.year, date.month, date.day,
                                                               date.hour, date.minute, date.second );
    if(msg_size >= (DATE_SIZE+1)) {
			return NULL_IDX;	
		}

    pantech_fseek( file, SEEK_SET, pos);
    res = pantech_fwrite( file, date_buf, DATE_SIZE );
    if( res != DATE_SIZE )
    {
        return NULL_IDX;
    }

    // Build Version
    ver_size = strlen( get_build_version() );
    size += ver_size;
    res = pantech_fwrite( file, (char*)get_build_version(), ver_size );
    if( res != ver_size )
    {
        return NULL_IDX;
    }

    // \r\n 삽입 : 로그발생 시각, 버전명과 로그 내용을 구분하기위해
    size += 2;
    res = pantech_fwrite( file, "\r\n", 2 );
    if( res != 2 )
    {
        return NULL_IDX;
    }

    // Msg
    msg_size = strlen( msg );
    if( msg_size > LOG_SIZE - size )
    {
        msg_size = LOG_SIZE - size;
    }
    size += msg_size;
    res = pantech_fwrite( file, (char*)msg, msg_size );
    if( res != msg_size )
    {
        return NULL_IDX;
    }

    // 나머지 부분 공백으로 padding
    res = pantech_fwrite( file, g_blank_buf + size, LOG_SIZE + 2 - size );
    if( res != LOG_SIZE + 2 - size )
    {
        return NULL_IDX;
    }

    return idx;
}


static BOOL get_log( IDX_T idx, char* log_buf )
{
    fs_handle_type  file;
    UINT32          pos;
    char*           iter;
    int             res;

    file = open_log_file();

    pos = g_head.first_log_pos_ + (UINT32)idx * ( 82 + 46 + 82 + LOG_SIZE + 2 + 2 );
    pantech_fseek( file, SEEK_SET, pos );
    res = pantech_fread( file, log_buf, LOG_SIZE ); 
    if( res != LOG_SIZE )
    {
        pantech_fclose( file );
        return FALSE;
    }

    for( iter = log_buf + LOG_SIZE - 2; *iter == ' '; --iter ) {}
    iter[ 1 ] = '\0';

    pantech_fclose( file );
    return TRUE;
}


/*********************************************************************
 * ERROR LOG ADD Routine
 * ******************************************************************/
static bool aarm_exception_status = FALSE;


void pantech_errlog_add_stack(const char *name, int size)
{
	int length;

	if(!aarm_exception_status) return;

	if(LogIdx >= LOG_SIZE - 1 - 1)
		return;

 
	if((LogIdx + size + 1) > LOG_SIZE - 1) 
		length = LOG_SIZE - 1 - LogIdx - 1; 
	else
		length = size;

	if(LogIdx != 0) {
		pErrorLog[LogIdx++] = ' ';
		pErrorLog[LogIdx++] = '<';
	}
	memcpy(&pErrorLog[LogIdx], name, length);
	LogIdx += length;
	pErrorLog[LogIdx++] = ' ';
}
EXPORT_SYMBOL(pantech_errlog_add_stack);

void pantech_errlog_add_log(const char *log, int size)
{
	int length;
	
	if(LogIdx >= LOG_SIZE - 2)
		return;

	if((LogIdx + size) > LOG_SIZE - 2) 
		length = LOG_SIZE - 2 - LogIdx; 
	else
		length = size;

	memcpy(&pErrorLog[LogIdx], log, length);
	LogIdx += length;
	if(log[length - 1] != '\n') {
		pErrorLog[LogIdx++] = '\n';
	}
}
EXPORT_SYMBOL(pantech_errlog_add_log);

void pantech_errlog_init_log(void)
{
	int length;
	char log[] = "\n[S]:";
	int size;

	if(LogIdx >= LOG_SIZE - 2)
		return;

	aarm_exception_status = TRUE;

	size = strlen(log);

	if((LogIdx + size) > LOG_SIZE - 2) 
		length = LOG_SIZE - 2 - LogIdx; 
	else
		length = size;

	memcpy(&pErrorLog[LogIdx], log, length);
	LogIdx += length;

	return;
}
EXPORT_SYMBOL(pantech_errlog_init_log);


void pantech_errlog_save_log(void)
{
	pantech_logfile_save_log(pErrorLog);
	return;
}
EXPORT_SYMBOL(pantech_errlog_save_log);

void pantech_errlog_noti_crash_to_marm(uint32_t reason)
{
	sky_sys_rst_NotiToMARM(&reason);
	return;
}
EXPORT_SYMBOL(pantech_errlog_noti_crash_to_marm);

/*****************************************************
 * ERROR LOG DISPLAY ROUTINE
 * **************************************************/
static atomic_t is_displayed = ATOMIC_INIT(0);
extern struct fb_info *registered_fb[FB_MAX];

static int cx, cy, cmaxx, cmaxy;
#ifdef F_SKYDISP_FRAMEBUFFER_32
static const unsigned int BGCOLOR = 0x00FF0000;
static const unsigned int FGCOLOR = 0x00FFFFFF;
#else
static const unsigned short BGCOLOR = 0x001F;
static const unsigned short FGCOLOR = 0xFFFF;
#endif

static const unsigned font5x12[] = {
  0x00000000, 0x00000000,
  0x08421080, 0x00020084,
  0x00052940, 0x00000000,
  0x15f52800, 0x0000295f,
  0x1c52f880, 0x00023e94,
  0x08855640, 0x0004d542,
  0x04528800, 0x000b2725,
  0x00021080, 0x00000000,
  0x04211088, 0x00821042,
  0x10841082, 0x00221108,
  0x09575480, 0x00000000,
  0x3e420000, 0x00000084,
  0x00000000, 0x00223000,
  0x3e000000, 0x00000000,
  0x00000000, 0x00471000,
  0x08844200, 0x00008442,
  0x2318a880, 0x00022a31,
  0x08429880, 0x000f9084,
  0x1108c5c0, 0x000f8444,
  0x1c4443e0, 0x00074610,
  0x14a62100, 0x000423e9,
  0x26d087e0, 0x00074610,
  0x1e10c5c0, 0x00074631,
  0x088443e0, 0x00010844,
  0x1d18c5c0, 0x00074631,
  0x3d18c5c0, 0x00074610,
  0x08e20000, 0x00471000,
  0x08e20000, 0x00223000,
  0x02222200, 0x00082082,
  0x01f00000, 0x000003e0,
  0x20820820, 0x00008888,
  0x1108c5c0, 0x00020084,
  0x2b98c5c0, 0x000f05b5,
  0x2318a880, 0x0008c63f,
  0x1d2949e0, 0x0007ca52,
  0x0210c5c0, 0x00074421,
  0x252949e0, 0x0007ca52,
  0x1e1087e0, 0x000f8421,
  0x1e1087e0, 0x00008421,
  0x0210c5c0, 0x00074639,
  0x3f18c620, 0x0008c631,
  0x084211c0, 0x00071084,
  0x10842380, 0x00032508,
  0x0654c620, 0x0008c525,
  0x02108420, 0x000f8421,
  0x2b5dc620, 0x0008c631,
  0x2b59ce20, 0x0008c739,
  0x2318c5c0, 0x00074631,
  0x1f18c5e0, 0x00008421,
  0x2318c5c0, 0x01075631,
  0x1f18c5e0, 0x0008c525,
  0x1c10c5c0, 0x00074610,
  0x084213e0, 0x00021084,
  0x2318c620, 0x00074631,
  0x1518c620, 0x0002114a,
  0x2b18c620, 0x000556b5,
  0x08a54620, 0x0008c54a,
  0x08a54620, 0x00021084,
  0x088443e0, 0x000f8442,
  0x0421084e, 0x00e10842,
  0x08210420, 0x00084108,
  0x1084210e, 0x00e42108,
  0x0008a880, 0x00000000,
  0x00000000, 0x01f00000,
  0x00000104, 0x00000000,
  0x20e00000, 0x000b663e,
  0x22f08420, 0x0007c631,
  0x22e00000, 0x00074421,
  0x23e84200, 0x000f4631,
  0x22e00000, 0x0007443f,
  0x1e214980, 0x00010842,
  0x22e00000, 0x1d187a31,
  0x26d08420, 0x0008c631,
  0x08601000, 0x00071084,
  0x10c02000, 0x0c94a108,
  0x0a908420, 0x0008a4a3,
  0x084210c0, 0x00071084,
  0x2ab00000, 0x0008d6b5,
  0x26d00000, 0x0008c631,
  0x22e00000, 0x00074631,
  0x22f00000, 0x0210be31,
  0x23e00000, 0x21087a31,
  0x26d00000, 0x00008421,
  0x22e00000, 0x00074506,
  0x04f10800, 0x00064842,
  0x23100000, 0x000b6631,
  0x23100000, 0x00022951,
  0x23100000, 0x000556b5,
  0x15100000, 0x0008a884, 
  0x23100000, 0x1d185b31,
  0x11f00000, 0x000f8444,
  0x06421098, 0x01821084,
  0x08421080, 0x00021084,
  0x30421083, 0x00321084,
  0x0004d640, 0x00000000,
  0x00000000, 0x00000000, 
};

#ifdef F_SKYDISP_FRAMEBUFFER_32
static void drawglyph(unsigned int *base, int pixels, unsigned int paint,
                      unsigned stride, const unsigned *glyph, int bpp)
{
    unsigned x, y, data;
    stride -= 5;

    data = glyph[0];
    for (y = 0; y < 6; y++) {
        for (x = 0; x < 5; x++) {
            if (data & 1) {
                base[pixels] = paint;
            }
            data >>= 1;
            pixels++;
        }
        pixels += stride;
    }
    data = glyph[1];
    for (y = 0; y < 6; y++) {
        for (x = 0; x < 5; x++) {
            if (data & 1) {
                base[pixels] = paint;
            }
            data >>= 1;
            pixels++;
        }
        pixels += stride;
    }
}                    
#else
static void drawglyph(unsigned char *base, int pixels, unsigned short paint,
                      unsigned stride, const unsigned *glyph, int bpp)
{
    unsigned x, y, data;
    stride -= 5;

    data = glyph[0];
    for(y = 0; y < 6; y++) {
        for(x = 0; x < 5; x++) {
            if(data & 1) {
                if(bpp == 2) {
                    ((unsigned short *)base)[pixels] = paint;
                } else if(bpp == 3){
                    base[pixels*3] = (paint & 0xF800) >> 11;
                    base[pixels*3 + 1] = (paint & 0x07E0) >> 5;
                    base[pixels*3 + 2] = (paint & 0x001F);
                }
            }
            data >>= 1;
            pixels++;
        }
        pixels += stride;
    }
    data = glyph[1];
    for(y = 0; y < 6; y++) {
        for(x = 0; x < 5; x++) {
            if(data & 1) {
                if(bpp == 2) {
                    ((unsigned short *)base)[pixels] = paint;
                } else if(bpp == 3){
                    base[pixels*3] = (paint & 0xF800) >> 11;
                    base[pixels*3 + 1] = (paint & 0x07E0) >> 5;
                    base[pixels*3 + 2] = (paint & 0x001F);
                }
            }
            data >>= 1;
            pixels++;
        } 
        pixels += stride;
    }
}
#endif

static void display_log(const char *str, int size)
{
    struct fb_info *info;
    unsigned short c;
    int i, j,k, count;
    int fb_width, fb_height;
    int fb_num;
    int bpp;
    int t_cx, t_cy;
    if (size <= 0) return;

    info = registered_fb[0];
    fb_width = info->var.xres;
    fb_height = info->var.yres;

    if (cx == 0 && cy == 0 && cmaxx == 0 && cmaxy == 0) { 
        cmaxx = fb_width / 6;
        cmaxy = (fb_height - 1) / 12;
        cx = cy = 0;
    }
    bpp = info->var.bits_per_pixel >> 3; // / 8;
    fb_num = info->fix.smem_len / (fb_width * fb_height * bpp);

    if (fb_num > 2 ) fb_num = 2;

    t_cx = t_cy = 0;
#ifdef F_SKYDISP_FRAMEBUFFER_32
    for (k = 0; k < fb_num; k++ ) {
        unsigned int *base;

        base  = ((unsigned int *)info->screen_base) + fb_width * fb_height * k;
        t_cx = cx; t_cy = cy;

        count = fb_width * 12;

        j = t_cy * fb_width * 12;

        while (count--) {                
            base[j++] = BGCOLOR;
        }

        for (i = 0; i < size; i++) {
            c = str[i];
            if (c > 127) continue;
            if (c < 32) {
                if (c == '\n') {
                    t_cy++;
                    t_cx = 0;
                    count = fb_width * 12;
                    j = t_cy * fb_width * 12;
                    while (count--) {                             
                        base[j++] = BGCOLOR;
                    }
                }
                continue;
            }
            drawglyph(base, t_cy * 12 * fb_width + t_cx * 6, FGCOLOR,
                    fb_width, font5x12 + (c - 32) * 2, bpp);
            t_cx++;
            if (t_cx >= cmaxx ) {
                t_cy++;
                t_cx = 0;
                count = fb_width * 12;
                j = t_cy * fb_width * 12;
                while (count--) {                        
                    base[j++] = BGCOLOR;
                }
            }
        }
    }
    cx = t_cx; cy = t_cy;
    cy++; cx= 0;
#else    
    if (bpp == 2) { // RGB565
        for(k = 0; k < fb_num; k++ ){
            unsigned short *base;

            base  = (unsigned short *)(((unsigned char *)info->screen_base) 
                    + fb_width * fb_height * bpp * k);
            t_cx = cx; t_cy = cy;

            count = fb_width * 12;  
            j = t_cy * fb_width * 12;
            while(count--) {
                base[j++] = BGCOLOR;
            }

            for(i = 0; i < size; i++) {
                c = str[i];
                if(c > 127) continue;
                if(c < 32){
                    if(c == '\n') {
                        t_cy++;
                        t_cx = 0;
                        count = fb_width * 12;
                        j = t_cy * fb_width * 12;
                        while(count--) {
                            base[j++] = BGCOLOR;
                        }
                    }
                    continue;
                }

                drawglyph((unsigned char *)base, t_cy * 12 * fb_width + t_cx * 6, FGCOLOR, 
                                 fb_width, font5x12 + (c - 32) * 2, bpp );
                t_cx++;
                if(t_cx >= cmaxx ){
                    t_cy++;
                    t_cx = 0;
                    count = fb_width * 12;
                    j = t_cy * fb_width * 12;
                    while(count--) {
                        base[j++] = BGCOLOR;
                    }
                }
            }

        }
        cx = t_cx; cy = t_cy;   

    } else { //RGB888
        for (k = 0; k < fb_num; k++ ) {
            unsigned char *base;

            base  = ((unsigned char *)info->screen_base) + fb_width * fb_height * bpp * k;
            t_cx = cx; t_cy = cy;

            count = fb_width * 12;  
            j = t_cy * fb_width * 12;
            while(count--) {
                base[j++] = (BGCOLOR & 0xF800) >> 11;
                base[j++] = (BGCOLOR & 0x07E0) >> 5;
                base[j++] = (BGCOLOR & 0x001F);
            }

            for(i = 0; i < size; i++) {
                c = str[i];
                if(c > 127) continue;
                if(c < 32){
                    if(c == '\n') {
                        t_cy++;
                        t_cx = 0;
                        count = fb_width * 12;
                        j = t_cy * fb_width * 12;
                        while(count--) {
                            base[j++] = (BGCOLOR & 0xF800) >> 11;
                            base[j++] = (BGCOLOR & 0x07E0) >> 5;
                            base[j++] = (BGCOLOR & 0x001F);
                        }
                    }
                    continue;
                }

                drawglyph((unsigned char*)base , t_cy * 12 * fb_width + t_cx * 6, FGCOLOR, 
                                 fb_width, font5x12 + (c - 32) * 2, bpp );
                t_cx++;
                if( t_cx >= cmaxx ){
                    t_cy++;
                    t_cx = 0;
                    count = fb_width * 12;
                    j = t_cy * fb_width * 12;
                    while(count--) {
                        base[j++] = (BGCOLOR & 0xF800) >> 11;
                        base[j++] = (BGCOLOR & 0x07E0) >> 5;
                        base[j++] = (BGCOLOR & 0x001F);
                    }
                }
            }
        }
        cx = t_cx; cy = t_cy;   
    }
    cy++; cx= 0;
#endif    
}


void pantech_errlog_display_add_log(const char *log, int size)
{
	int length;
	if(size <= 0 || DisplayLogIdx >= DISPLAY_LOG_SIZE) return;
	
	if((DisplayLogIdx + size) >= DISPLAY_LOG_SIZE) {
		length = DISPLAY_LOG_SIZE - DisplayLogIdx;
	} else {
		length = size;
	}

    memcpy(&pDisplayBuffer[DisplayLogIdx], log, length);
    DisplayLogIdx += length;
 	if(pDisplayBuffer[DisplayLogIdx - 1] != '\n') {
 		pDisplayBuffer[DisplayLogIdx++] = '\n';
	}
}
EXPORT_SYMBOL(pantech_errlog_display_add_log);

void pantech_errlog_display_put_log(const char *log, int size)
{
	display_log(log, size);
}
EXPORT_SYMBOL(pantech_errlog_display_put_log);

void pantech_errlog_display_put_smemlog(void)
{
	if(LogIdx == 0 && (pErrorLog[0] != 0x00)) {
		pErrorLog[LOG_SIZE - 1] = 0x00;
		LogIdx = strlen(pErrorLog);
	}

	display_log(pErrorLog, LogIdx);
}
EXPORT_SYMBOL(pantech_errlog_display_put_smemlog);

#define WDT0_RST	(MSM_TMR0_BASE + 0x38)
void pantech_errlog_display_with_errlog(bool bmArm, bool do_panic)
{
    char *str;

	if (atomic_read(&is_displayed) == 0) {
		atomic_set(&is_displayed, 1);

		if (do_panic) {
			panic("go to panic state by SW");
		}
	}

	return;
}
EXPORT_SYMBOL(pantech_errlog_display_with_errlog);

/*************************************************************************
 * END DISPLAY ROUTINE ***************************************************
 * **********************************************************************/

/*******************************************************************
 * pantech_errlog device driver routine 
 * ****************************************************************/

static ssize_t pantech_errlog_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	return 0;
}

//fatal:, normal:
static ssize_t pantech_errlog_write(struct file *fp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	char cmd[6];
	int len;

	if (count < 1)
		return 0;

	if (copy_from_user(cmd, buf, 6))
		return -EFAULT;
	
	if(strncmp(cmd, "fatal:", 6) != 0) {
//	if(cmd == SYS_RESET_REASON_ANDROID) {
		if(count > LOG_SIZE) {
			len = LOG_SIZE;
		} else len = count;

		if(copy_from_user(&pErrorLog[0], buf, len))
			return -EFAULT;
		LogIdx = len;
        pantech_errlog_display_add_log("android system error\n",
                strlen("android system error\n"));
        pantech_errlog_noti_crash_to_marm(SYS_RESET_REASON_ANDROID);
		pantech_errlog_display_with_errlog(false, true);	
	} else if(strncmp(cmd, "norml:", 6) != 0) {
//	else if(cmd == SYS_RESET_REASON_NORMAL) {
		if((count) > LOG_SIZE) {
			len = LOG_SIZE;
		} else len = count;

		if(copy_from_user(&pErrorLog[0], buf, len))
			return -EFAULT;
		LogIdx = len;

		pantech_errlog_save_log();
	} else {
		//no action
	}
	
	return count;
}

static int pantech_errlog_open(struct inode *ip, struct file *fp)
{
	return 0;
}

static int pantech_errlog_release(struct inode *ip, struct file *fp)
{
	return 0;
}

static const struct file_operations pantech_errlog_fops = {
	.owner = THIS_MODULE,
	.read = pantech_errlog_read,
	.write = pantech_errlog_write,
	.open = pantech_errlog_open,
	.release = pantech_errlog_release,
};

static struct miscdevice pantech_errlog_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "pantech_errlog",
	.fops = &pantech_errlog_fops,
};


static int __init pantech_errlog_init(void)
{
	pErrorLog = sky_sys_rst_GetLogBuffer();
	LogIdx = 0; 
    pDisplayBuffer = kmalloc(DISPLAY_LOG_SIZE, GFP_KERNEL);
    DisplayLogIdx = 0;

	memset(pErrorLog, 0x00, LOG_SIZE);
    memset(pDisplayBuffer, 0x00, DISPLAY_LOG_SIZE);
    cx = cy = cmaxx = cmaxy = 0;

	return misc_register(&pantech_errlog_dev);
}

module_init(pantech_errlog_init);

MODULE_DESCRIPTION("Pantech Error Log");
MODULE_LICENSE("GPL v2");
