#ifndef __RCM_MB115_01_BOOT_H_INCLUDED_
#define __RCM_MB115_01_BOOT_H_INCLUDED_

#define RUMBOOT_HEADER_MAGIC 0xdeadc0de

struct __attribute__((packed)) rumboot_header
{
	uint32_t magic; // magic
	uint32_t length; // filled by rumboot-packimage
	uint32_t entry_point_0; // entry 0
	uint32_t entry_point_1; // entry 0
	uint32_t checksum_1; // filled by rumboot-packimage
	uint32_t checksum_2; // filled by rumboot-packimage
	uint8_t data[];
};

#endif // !__RCM_MB115_01_BOOT_H_INCLUDED_
