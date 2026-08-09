target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ./ST7735 ./ST7789)
file(GLOB ST7735_SOURCES CONFIGURE_DEPENDS ./ST7735/*.c)
file(GLOB ST7789_SOURCES CONFIGURE_DEPENDS ./ST7789/*.c)

# The ST7789 fonts.c file is fully disabled with #if 0.  Both LCD wrappers
# share the active ASCII tables from the ST7735 font.c file.
list(FILTER ST7789_SOURCES EXCLUDE REGEX "/fonts\\.c$")

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${ST7735_SOURCES} ${ST7789_SOURCES})
