#include "athena_status_led_driver/terminal_transport.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sys/ioctl.h>
#include <unistd.h>

namespace athena_status_led_driver
{

bool TerminalTransport::open()
{
  open_ = true;
  // Hide cursor and clear screen
  std::printf( "\033[?25l" );
  std::printf( "\033[2J" );
  std::fflush( stdout );
  return true;
}

void TerminalTransport::close()
{
  open_ = false;
  // Show cursor and reset colors
  std::printf( "\033[?25h\033[0m\n" );
  std::fflush( stdout );
}

bool TerminalTransport::isOpen() const { return open_; }

bool TerminalTransport::send( const std::vector<Color> &leds )
{
  if ( !open_ || leds.empty() )
    return false;

  // Get terminal size
  struct winsize w;
  int term_cols = 80;
  int term_rows = 24;
  if ( ioctl( STDOUT_FILENO, TIOCGWINSZ, &w ) != -1 ) {
    if ( w.ws_col > 0 && w.ws_row > 0 ) {
      term_cols = w.ws_col;
      term_rows = w.ws_row;
    }
  }

  // Reserve a few rows for status text and leave small side padding
  int available_rows = std::max( 5, term_rows - 3 );
  int available_cols = std::max( 10, term_cols - 2 );

  // Characters are roughly 2:1 aspect ratio (twice as tall as wide).
  // Thus we want x_radius = 2 * y_radius to look like a circle.
  int ring_radius_y = std::min( ( available_rows - 2 ) / 2, ( available_cols - 2 ) / 4 );
  int ring_radius_x = ring_radius_y * 2;

  if ( ring_radius_y < 1 )
    ring_radius_y = 1;
  if ( ring_radius_x < 2 )
    ring_radius_x = 2;

  int center_x = ring_radius_x + 1;
  int center_y = ring_radius_y + 1;
  int grid_width = center_x * 2 + 1;
  int grid_height = center_y * 2 + 1;

  // Build a grid of characters — each cell is either a colored '●' or a space
  struct Cell {
    bool occupied = false;
    uint8_t r = 0, g = 0, b = 0;
  };
  std::vector<std::vector<Cell>> grid( grid_height, std::vector<Cell>( grid_width ) );

  // Place each LED on the ring
  const size_t led_count = leds.size();
  for ( size_t i = 0; i < led_count; ++i ) {
    // Angle: LED 0 at top, clockwise
    double angle =
        2.0 * M_PI * static_cast<double>( i ) / static_cast<double>( led_count ) - M_PI / 2.0;
    int x = center_x + static_cast<int>( std::round( ring_radius_x * std::cos( angle ) ) );
    int y = center_y + static_cast<int>( std::round( ring_radius_y * std::sin( angle ) ) );

    if ( x >= 0 && x < grid_width && y >= 0 && y < grid_height ) {
      grid[y][x].occupied = true;
      grid[y][x].r = leds[i].r;
      grid[y][x].g = leds[i].g;
      grid[y][x].b = leds[i].b;
    }
  }

  // Move cursor to top-left
  std::printf( "\033[H" );

  // Render the grid
  for ( int y = 0; y < grid_height; ++y ) {
    for ( int x = 0; x < grid_width; ++x ) {
      if ( grid[y][x].occupied ) {
        std::printf( "\033[38;2;%d;%d;%dm●", grid[y][x].r, grid[y][x].g, grid[y][x].b );
      } else {
        std::printf( " " );
      }
    }
    // \033[0m: reset color
    // \033[K: clear to end of line (removes artifacts from shrinking width)
    std::printf( "\033[0m\033[K\n" );
  }

  // \033[J: clear from cursor to end of screen (removes artifacts from shrinking height)
  std::printf( "\033[J" );

  std::fflush( stdout );
  return true;
}

} // namespace athena_status_led_driver
