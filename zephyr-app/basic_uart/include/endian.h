#include <zephyr/types.h>

inline uint16_t htole16(uint16_t data)
{
    return sys_cpu_to_le16(data);
}


inline uint32_t htole32(uint32_t data)
{
    return sys_cpu_to_le32(data);
}

inline uint64_t htole64(uint64_t data)
{
    return sys_cpu_to_le64(data);
}

inline uint16_t le16toh(uint16_t data)
{
    return sys_le16_to_cpu(data);
}


inline uint32_t le32toh(uint32_t data)
{
    return sys_le32_to_cpu(data);
}

inline uint64_t le64toh(uint64_t data)
{
    return sys_le64_to_cpu(data);
}
