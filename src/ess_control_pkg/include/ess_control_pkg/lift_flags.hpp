#ifndef ESS_CONTROL_PKG_LIFT_FLAGS_H__
#define ESS_CONTROL_PKG_LIFT_FLAGS_H__

#include <cstdint>

namespace ess{
	constexpr uint32_t GODOWN =  0;
	constexpr uint32_t GOUP = (1u << 0 );
	constexpr uint32_t TOFLOOR1 = (1u << 1);
	constexpr uint32_t TOFLOOR2 = (1u << 2);
	constexpr uint32_t TOFLOOR3 = (1u << 3);
}
#endif
