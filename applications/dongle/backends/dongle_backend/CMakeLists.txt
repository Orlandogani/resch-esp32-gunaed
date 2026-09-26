# The backend vtable. Header-only: the contract main/ and every backends/<name>/ agree
# on (ADR-013). ringbuf is public because the resources carry ringbuf_t pointers.
idf_component_register(INCLUDE_DIRS "include"
                       REQUIRES ringbuf)
