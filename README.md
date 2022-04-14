# Waveshare 4.2 inch grayscale - waveshare4in2gsc

A GFX enabled device driver for the Waveshare 4.2 inch grayscale e-paper display

This library allows GFX to bind to a Waveshare 4.2 inch grayscale display so that you can use it as a draw target.

Documentation for GFX is here: https://www.codeproject.com/Articles/5302085/GFX-Forever-The-Complete-Guide-to-GFX-for-IoT

To use GFX, you need GNU C++14 enabled. You also need to include the requisite library. Your platformio.ini should look something like this:

```
[env:node32s]
platform = espressif32
board = node32s
framework = arduino
lib_deps = 
	codewitch-honey-crisis/htcw_waveshare4in2gsc@^0.9.6
lib_ldf_mode = deep
build_unflags=-std=gnu++11
build_flags=-std=gnu++14
```

Unlike other drivers, this driver is not a draw target.

It exposes modes through the mode<>() template function that can be used to retrieve draw surfaces. Keep in mind that each one requires its own frame buffer, so it's best to keep only one around at once. The template parameter specifies the virtualized bitdepth for monochrome display mode, or 0 to choose the 4-color natural grayscale mode

Note: Prerelease. Partial update is supported by the display itself but is not working in the driver. When it does end up working it will not require a code change.