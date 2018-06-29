#ifndef _ENTER_H_
#define _ENTER_H_
#include "stdint.h"

/*
    FWD �ļ�Ϊɽ��UD����İ�ר�õĹ̼��ļ����ṩ�����ֿ�ͺ������ܣ��ӿ쿪���ٶȺ͵����ٶȡ�
    FWD�ǿ�ִ�е��ļ���ֱ����¼��ָ����λ�ã������� APP�е�����ص� API ��������UD�棬����ʹ��FWD�̼���

    ��¼������

        ���usb�ߣ����� ��������(Ĭ���� PTD7�������İ��ϵĶ�������)��Ȼ��λ�����ɽ���UDģʽ��

        ���Ի�ʶ�𵽲���UD�����̣���FWD�ļ����Ƶ�U�̵ĸ�Ŀ¼���ɡ�


    ����ʹ���з�����bug���뷢�ʼ��������䣺minimcu@foxmail.com�����ǻᾡ����¡�

*/


//�û���API �����ӿڣ��ɿ�����Ľṹ���ע�ͣ���ע������ԭ��
//һ�㿴ע����ĺ���ԭ�ͼ���ʹ�õġ�
//APP��Ҫ��ʼ��ʱִ�� ASSERT(enter_init());�ж��Ƿ��Ѿ���¼�˹̼�

#define FWD_VER     103

typedef struct
{
    uint16_t    flag;       //���
    uint16_t    ver;        //�汾

    //�ֿ�
    uint8_t *   ascii;      //Ӣ���ֿ�      6x12
    uint8_t *   chinese;    //�����ֿ����  12x12

    //����С����
    void *      cpy;        //void *    memcpy  ( uint8_t *dst, uint8_t *src, uint32_t count)       �ڴ渴��
    void *      memset;     //void *    memset  (void *src, int c, int count);                      �ڴ�������ֵ
    void *      img_ext;    //void      img_extract(void *dst, void *src, uint32_t srclen);         ����ͷ��ѹ

    //�ַ��� ���� ת��
    void *      sptf;       //int       sprintf (char *buf, const char *fmt, ...)
    void *      ptf;        //void      printf  (fp,fmt, ...)
    void *      atoi;       //int       atoi    (char *str);                                        �ַ���ת������
    void *      itoa;       //char *    itoa    (int n, char * chBuffer);                           ����ת��Ϊ�ַ���
    void *      ftoa;       //char *    ftoa    (double dValue, char * chBuffer);                   ����ת��Ϊ�ַ���

    void *      get_isr;    //int       get_isr_info(char * str);

    //flash API
    void *      flash_init;         //void    flash_init();                                 ��ʼ��Flash
    void *      flash_erase_sector; //uint8   flash_erase_sector  (uint16 sectorNum);             //����ָ��flash����
    void *      flash_write;        //uint8   flash_write         (uint16 sectorNum, uint16 offset, FLASH_WRITE_TYPE data);                //д��flash����
    void *      flash_write_buf;    //uint8   flash_write_buf     (uint16 sectorNum, uint16 offset, uint16 cnt, uint8 buf[]);              //�ӻ�����д��flash����
    //����һ��flash API�ӿڣ�                  flash_read          (sectorNum,offset,type)


    //flash �洢����
    void *      flash_data_init;    //void      flash_data_init     (flash_data_t *d);      //�Խṹ�����ݽ���Ԥ������flash�����ݻ�û��ʼ����������flash
    void *      flash_data_reset;   //void      flash_data_reset    (flash_datasave_t *d);  //����flash�����ǶԴ洢���ݽ������
    void *      flash_data_load;    //uint8_t   flash_data_load     (flash_datasave_t *d);  //�� flash ��������
    void *      flash_data_save;    //uint8_t   flash_data_save     (flash_datasave_t *d);  //�������ݵ�flash

    //ִ����APP API�ӿ� ������ͨ�� �������ò�ͬ������ĺ���������ջ��ַ��
    void *      jmp_app;            //void      jmp_app(uint32_t sp, uint32_t pc);          //��ת���µ�APP
    void *      jmp_bin;            //void      jmp_bin(uint32_t addr);                     //��ת���µ�APP BIN�ļ� ��bin�ļ��ĵ�1��32λ������ջ��ַ����2��32λ����������������ڣ�


}mydata_in_t;


//���ݵĳߴ� ������ FLASH_ALIGN_ADDR ����
//
typedef struct
{
    uint16_t    sectornum_start;            //��ʼ������
    uint16_t    sectornum_end;              //����������
    void *     data_addr;                   //���ݵĵ�ַ
    uint16_t    data_size;                  //���ݵĳߴ�
    uint16_t    res1;                       //�������ײ�ʹ�� ����ɾ��
    uint32_t    res2[2];                    // �������ײ�ʹ�� ����ɾ��


}flash_data_t;

//     �̼�����ʼ��ַ (�û����ɸ��ģ�����APP����ʶ��)
#define START_APP_ADDR  (32*1024)               //APP������ַ
#if defined(MK60DZ10)
#define START_FWD_ADDR  (256*1024)              //DN ����¼����벿��flash��256k
#elif defined(MK60FN15)
#define START_FWD_ADDR  (512*1024)              //FN ����¼����벿��flash��512k
#elif defined(MK60FX15)
#define START_FWD_ADDR  (0x10000000)            //FX ����¼�� flexNVM
#endif

//�ڲ��ã��û������ע
#define OFFSET(type, member)    (uint32)(&(((type *)0)->member))
#define IN_ADDR(member)         (*((uint32_t *)(START_FWD_ADDR + OFFSET(mydata_in_t, member)) ))


//�û��� API �ӿڵ���
//
#ifndef FWD_LIB         //�����ڹ����ж���˺궨��(�ڲ�ʶ���õ�) by ɽ��

#define  enter_init()      (((uint16_t)IN_ADDR(flag)==0x55A5) && (uint16_t)IN_ADDR(ver)==FWD_VER)      //�����жϹ̼��ļ��Ƿ���� ,APP��Ҫ��ʼ��ʱִ�� ASSERT(enter_init());

#define  ASCII              ((uint8_t *)(IN_ADDR(ascii)))              // ASCII�ֿ� (�ߴ翴�ṹ��mydata_in_t��ע��)
#define  CHINESE            ((uint8_t *)(IN_ADDR(chinese)))            // �����ֿ�  (�ߴ翴�ṹ��mydata_in_t��ע��)

//�ڴ渴�ƺ���
typedef void * (*memcpy_t)( uint8_t *dst, uint8_t *src, uint32_t count);
#define memcpy(dst,src,n)                       (*(memcpy_t)(IN_ADDR(cpy)))((uint8_t *)dst,(uint8_t *)src,n)

typedef void *    (*memset_t)  (void *src, int c, int count);
#define memset(src,c,n)                       (*(memset_t)(IN_ADDR(memset)))((void *)src,c,n)

//ͼ���ѹ����
typedef void (*img_ext_t)(void *dst, void *src, uint32_t srclen);
#define  img_extract(dst,src,srclen)            (*(img_ext_t)(IN_ADDR(img_ext)))(dst,src,srclen)

//sprintf������ת��Ϊ�ַ���
typedef int (*sprintf_t)(char *buf, const char *fmt, ...);
#define sprintf(buf,fmt, ...)                   (*(sprintf_t)(IN_ADDR(sptf)))(buf,fmt, ## __VA_ARGS__)

//printf����ӡ��������Ҫ�Լ�ʵ�� enter_fputc ����
extern int enter_fputc(char ch);                    //��Ҫ�û��Զ���ʵ�ִ˺���
typedef int fputc_t (char c);
typedef int (*printf_t)(fputc_t *fp, const char *fmt, ...);
#define printf(fmt, ...)                        (*(printf_t)(IN_ADDR(ptf)))(enter_fputc,fmt, ## __VA_ARGS__)

//��ȡ�ж���ʾ��Ϣ���ĸ��˿ڴ����жϣ�
typedef int (*get_isr_info_t)(char * str);
#define  get_isr_info(str)                      (*(get_isr_info_t)(IN_ADDR(get_isr)))(str)

//�ַ���ת������
typedef int (*atoi_t)(char * str);
#define  atoi(str)                      (*(atoi_t)(IN_ADDR(atoi)))(str)

//����ת��Ϊ�ַ���
typedef char * (*itoa_t)(int n, char * str);
#define  itoa(n,str)                      (*(itoa_t)(IN_ADDR(itoa)))(n,str)     //int n=123;char buff[20]; itoa(n,buff);   //�ѱ���123ת��Ϊ�ַ���

//����ת��Ϊ�ַ���
typedef char * (*ftoa_t)(double dValue, char * str);
#define  ftoa(d,str)                      (*(ftoa_t)(IN_ADDR(ftoa)))(d,str)     //


// ****************** FLASH API ר�� ******************
#if defined(MK60DZ10)
#define     FLASH_SECTOR_SIZE       (2*1024)                //������С Ϊ 2k �ֽ�
#define     FLASH_SECTOR_NUM        (256)                   //������

#define     FLASH_ALIGN_ADDR        4                       //��ַ����������
typedef     uint32                  FLASH_WRITE_TYPE;       //flash_write ����д�� ����������


#elif defined(MK60FX15)
#define     FLASH_SECTOR_SIZE       (4*1024)                //������С Ϊ 4k �ֽ�
#define     FLASH_SECTOR_NUM        (128)                   //������

#define     FLASH_ALIGN_ADDR        8                       //��ַ����������
typedef     uint64                  FLASH_WRITE_TYPE;       //flash_write ����д�� ����������

#elif defined(MK60FN15)
#define     FLASH_SECTOR_SIZE       (4*1024)                //������С Ϊ 4k �ֽ�
#define     FLASH_SECTOR_NUM        (256)                   //������

#define     FLASH_ALIGN_ADDR        8                       //��ַ����������
typedef     uint64                  FLASH_WRITE_TYPE;       //flash_write ����д�� ����������
#endif

// ��ʼ��Flash
typedef void  (*flash_init_t)(void);
#define  flash_init()                      (*(flash_init_t)(IN_ADDR(flash_init)))()     //

// ����ָ��flash����
typedef uint8  (*flash_erase_sector_t)(uint16 sectorNum);
#define  flash_erase_sector(sectorNum)     (*(flash_erase_sector_t)(IN_ADDR(flash_erase_sector)))(sectorNum)

// д��flash����
typedef uint8  (*flash_write_t)(uint16 sectorNum, uint16 offset, FLASH_WRITE_TYPE data);
#define  flash_write(sectorNum,offset,data)  (*(flash_write_t)(IN_ADDR(flash_write)))(sectorNum,offset,data)

// д��flash����
typedef uint8  (*flash_write_buf_t)(uint16 sectorNum, uint16 offset, uint16 cnt, uint8 buf[]);
#define  flash_write_buf(sectorNum,offset,cnt,buf)  (*(flash_write_buf_t)(IN_ADDR(flash_write_buf)))(sectorNum,offset,cnt,buf)

#define     flash_off(sectorNo,offset)              ((uint32)(((sectorNo)*FLASH_SECTOR_SIZE) + (offset)))
#define     flash_read(sectorNo,offset,type)        (*(type *)flash_off(sectorNo,offset) )         //��ȡ����

//flash �洢����

//�Խṹ�����ݽ���Ԥ������flash�����ݻ�û��ʼ����������flash (�� flash��ʼ��)
typedef void  (*flash_data_init_t)(flash_data_t *d);
#define  flash_data_init(d)                      (*(flash_data_init_t)(IN_ADDR(flash_data_init)))(d)     //

//����flash�����ǶԴ洢���ݽ������
typedef void  (*flash_data_reset_t)(flash_data_t *d);
#define  flash_data_reset(d)                      (*(flash_data_reset_t)(IN_ADDR(flash_data_reset)))(d)     //

//�� flash ��������
typedef uint8_t  (*flash_data_load_t)(flash_data_t *d);
#define  flash_data_load(d)                      (*(flash_data_load_t)(IN_ADDR(flash_data_load)))(d)     //

//�������ݵ�flash
typedef uint8_t  (*flash_data_save_t)(flash_data_t *d);
#define  flash_data_save(d)                      (*(flash_data_save_t)(IN_ADDR(flash_data_save)))(d)     //

//��תִ����APP
typedef void  (*jmp_app_t)(uint32_t sp, uint32_t pc);
#define  jmp_app(sp,pc)                      (*(jmp_app_t)(IN_ADDR(jmp_app)))(sp,pc)

//��תִ����APP bin��ַ����jmp_app��ͬ�ĵط��ǣ�bin��ַ��һ��32λ������sp���ڶ���32λ������PC����
typedef void  (*jmp_bin_t)(uint32_t addr);
#define  jmp_bin(d)                      (*(jmp_bin_t)(IN_ADDR(jmp_bin)))(d)



#endif    //

#endif   //_ENTER_H_

