idf_component_register(SRCS "test_usb_device.c"
                       INCLUDE_DIRS "."
                       REQUIRES unity usb_device usb_hid usb_audio ringbuf power pm_policy)
