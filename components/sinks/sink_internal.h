/**
 * @file    sink_internal.h
 * @brief   Private shared definitions for the sinks component (NOT public ABI).
 *
 * @details
 *   The public sinks.h exposes ::sink_handle_t as an opaque `struct sink_s *`
 *   and ::sink_vtable_t as the per-sink interface. This internal header gives the
 *   concrete sink implementations (sink_debug.c, sink_gdl90.c) and the registry
 *   (sinks.c) a single agreed definition of `struct sink_s`, so a constructed
 *   handle and the registry agree on layout. It is deliberately kept out of the
 *   public include/ directory — nothing outside this component may depend on it.
 *
 * @copyright Novabox / ADSBin. Receive-only, experimental, non-certified.
 */
#pragma once

#include "sinks.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Concrete sink instance behind the opaque ::sink_handle_t.
 *
 * @details
 *   A sink is just its vtable plus a tiny amount of registry bookkeeping. The
 *   vtable's @c ctx carries each concrete sink's private state (allocated by its
 *   *_create()), and the function pointers route publish/feed/destroy to it. The
 *   registry only ever touches this struct's @c vt; concrete code only ever
 *   touches @c vt.ctx.
 */
struct sink_s {
    sink_vtable_t vt;          /**< The behavior + private ctx for this sink.    */
    bool          registered;  /**< true while linked into the publisher list.   */
};

#ifdef __cplusplus
}
#endif
