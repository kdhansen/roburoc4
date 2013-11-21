# - Find the curses include file and library
#
# Very basic find module, because the one shipped with cmake
# didn't work very well.

find_library(Curses_curses_LIBRARY NAMES curses)
find_library(Curses_ncurses_LIBRARY NAMES ncurses)
find_library(Curses_menu_LIBRARY NAMES menu)
set(Curses_LIBRARIES ${Curses_curses_LIBRARY} ${Curses_ncurses_LIBRARY} ${Curses_menu_LIBRARY})
find_path(Curses_INCLUDE_DIRS ncurses.h curses.h menu.h)
