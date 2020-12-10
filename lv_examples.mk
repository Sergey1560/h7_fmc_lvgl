C_SOURCES += $(shell find -L  Lib/lvgl_examples -name \*.c)


C_INCLUDES +=  \
-ILib/lvgl_examples \
-ILib/lvgl_examples/src \
-ILib/lvgl_examples/src/lv_demo_benchmark \
-ILib/lvgl_examples/src/lv_demo_keypad_encoder \
-ILib/lvgl_examples/src/lv_demo_printer \
-ILib/lvgl_examples/src/lv_demo_stress \
-ILib/lvgl_examples/src/lv_demo_widgets \
-ILib/lvgl_examples/src/lv_demo_widgets \
-ILib/lvgl_examples/src/lv_ex_get_started \
-ILib/lvgl_examples/src/lv_ex_style \
-ILib/lvgl_examples/src/lv_ex_widgets