#ifndef DEBUG_H
#define DEBUG_H

//#include <assert.h>

#ifndef NDEBUG
#define debug(fmt,args...) printk(KERN_INFO "[%s:%s:%d] " fmt "\n", __FILE__,__FUNCTION__,__LINE__, ## args)
#else
#define debug(fmt, args...)
#endif

//#define info(fmt,args...) printf("[%s:%s:%d] " fmt "\n", __FILE__,__FUNCTION__,__LINE__, ## args)
    

//void hexdump(const unsigned char *buf, unsigned short len);

#endif //DEBUG_H
