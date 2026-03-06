#include <gtest/gtest.h>

#include "./mock_transport.hpp"
#include "athena_status_led_driver/effects/battery_connection_effect.hpp"
#include "athena_status_led_driver/effects/battery_pulse_effect.hpp"
#include "athena_status_led_driver/effects/operating_mode_effect.hpp"
#include "athena_status_led_driver/effects/power_supply_effect.hpp"
#include "athena_status_led_driver/effects/rainbow_loading_effect.hpp"
#include "athena_status_led_driver/effects/spot_light_effect.hpp"
#include "athena_status_led_driver/led_effect.hpp"
#include "athena_status_led_driver/led_ring_controller.hpp"

using namespace athena_status_led_driver;

static constexpr size_t LED_COUNT = 110;

// ============================================================================
// Color tests
// ============================================================================

TEST( ColorTest, DefaultIsBlack )
{
  Color c;
  EXPECT_EQ( c.r, 0 );
  EXPECT_EQ( c.g, 0 );
  EXPECT_EQ( c.b, 0 );
}

TEST( ColorTest, Equality )
{
  Color a( 10, 20, 30 );
  Color b( 10, 20, 30 );
  Color c( 10, 20, 31 );
  EXPECT_EQ( a, b );
  EXPECT_NE( a, c );
}

TEST( ColorTest, ScaleFullReturnsOriginal )
{
  Color c( 100, 200, 50 );
  EXPECT_EQ( c.scaled( 1.0f ), c );
}

TEST( ColorTest, ScaleZeroReturnsBlack )
{
  Color c( 100, 200, 50 );
  EXPECT_EQ( c.scaled( 0.0f ), Color( 0, 0, 0 ) );
}

TEST( ColorTest, ScaleHalf )
{
  Color c( 100, 200, 50 );
  Color scaled = c.scaled( 0.5f );
  EXPECT_EQ( scaled.r, 50 );
  EXPECT_EQ( scaled.g, 100 );
  EXPECT_EQ( scaled.b, 25 );
}

TEST( ColorTest, BlendOverFullAlpha )
{
  Color base( 100, 100, 100 );
  Color result = base.blendOver( Color( 255, 0, 0 ), 1.0f );
  EXPECT_EQ( result, Color( 255, 0, 0 ) );
}

TEST( ColorTest, BlendOverZeroAlpha )
{
  Color base( 100, 100, 100 );
  Color result = base.blendOver( Color( 255, 0, 0 ), 0.0f );
  EXPECT_EQ( result, base );
}

// ============================================================================
// MockTransport tests
// ============================================================================

TEST( MockTransportTest, OpenClose )
{
  MockTransport transport;
  EXPECT_FALSE( transport.isOpen() );
  EXPECT_TRUE( transport.open() );
  EXPECT_TRUE( transport.isOpen() );
  transport.close();
  EXPECT_FALSE( transport.isOpen() );
}

TEST( MockTransportTest, SendRecordsFrame )
{
  MockTransport transport;
  transport.open();

  std::vector<Color> frame( 3, Color( 255, 0, 0 ) );
  EXPECT_TRUE( transport.send( frame ) );
  EXPECT_EQ( transport.frameCount(), 1u );
  EXPECT_EQ( transport.lastFrame().size(), 3u );
  EXPECT_EQ( transport.lastFrame()[0], Color( 255, 0, 0 ) );
}

// ============================================================================
// OperatingModeEffect tests
// ============================================================================

TEST( OperatingModeEffectTest, InactiveByDefault )
{
  OperatingModeEffect effect;
  EXPECT_FALSE( effect.isActive() );
}

TEST( OperatingModeEffectTest, SetValidMode )
{
  OperatingModeEffect effect;
  effect.setMode( "autonomous" );
  EXPECT_TRUE( effect.isActive() );
  EXPECT_EQ( effect.currentColor(), Color( 0, 80, 255 ) );
}

TEST( OperatingModeEffectTest, AllModeColors )
{
  OperatingModeEffect effect;

  effect.setMode( "autonomous" );
  EXPECT_EQ( effect.currentColor(), Color( 0, 80, 255 ) );

  effect.setMode( "driving" );
  EXPECT_EQ( effect.currentColor(), Color( 255, 180, 0 ) );

  effect.setMode( "manipulation" );
  EXPECT_EQ( effect.currentColor(), Color( 255, 60, 0 ) );

  effect.setMode( "safe" );
  EXPECT_EQ( effect.currentColor(), Color( 0, 255, 0 ) );
}

TEST( OperatingModeEffectTest, InvalidModeIgnored )
{
  OperatingModeEffect effect;
  effect.setMode( "autonomous" );
  Color before = effect.currentColor();
  effect.setMode( "invalid_mode" );
  EXPECT_EQ( effect.currentColor(), before );
  EXPECT_FALSE( effect.isModeValid( "invalid_mode" ) );
}

TEST( OperatingModeEffectTest, RenderFillsAllPixels )
{
  OperatingModeEffect effect;
  effect.setMode( "safe" );

  std::vector<Color> pixels( LED_COUNT );
  effect.render( pixels );

  for ( const auto &p : pixels )
    EXPECT_EQ( p, Color( 0, 255, 0 ).scaled( OperatingModeEffect::BRIGHTNESS ) );
}

// ============================================================================
// BatteryPulseEffect tests
// ============================================================================

TEST( BatteryPulseEffectTest, InactiveByDefault )
{
  BatteryPulseEffect effect;
  EXPECT_FALSE( effect.isActive() );
}

TEST( BatteryPulseEffectTest, LowBattery_BothLow )
{
  BatteryPulseEffect effect;
  std::array<uint16_t, 8> cells1 = { 3600, 4000, 4000, 4000, 4000, 4000, 4000, 4000 };
  std::array<uint16_t, 8> cells2 = { 4000, 4000, 4000, 3600, 4000, 4000, 4000, 4000 };
  effect.updateBatteryState( cells1, cells2 );
  EXPECT_TRUE( effect.isLowBattery() );
  EXPECT_TRUE( effect.isActive() );
}

TEST( BatteryPulseEffectTest, NotLowBattery_OnlyOneLow )
{
  BatteryPulseEffect effect;
  std::array<uint16_t, 8> cells1 = { 3600, 4000, 4000, 4000, 4000, 4000, 4000, 4000 };
  std::array<uint16_t, 8> cells2 = { 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000 };
  effect.updateBatteryState( cells1, cells2 );
  EXPECT_FALSE( effect.isLowBattery() );
}

TEST( BatteryPulseEffectTest, NotLowBattery_NeitherLow )
{
  BatteryPulseEffect effect;
  std::array<uint16_t, 8> cells1 = { 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000 };
  std::array<uint16_t, 8> cells2 = { 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000 };
  effect.updateBatteryState( cells1, cells2 );
  EXPECT_FALSE( effect.isLowBattery() );
}

TEST( BatteryPulseEffectTest, ZeroCellVoltagesAreIgnored )
{
  BatteryPulseEffect effect;
  std::array<uint16_t, 8> cells1 = { 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000 };
  std::array<uint16_t, 8> cells2 = { 0, 0, 0, 0, 0, 0, 0, 0 };
  effect.updateBatteryState( cells1, cells2 );

  EXPECT_FALSE( effect.isLowBattery() );
}

TEST( BatteryPulseEffectTest, PulseAdvancesPhase )
{
  BatteryPulseEffect effect;
  double before = effect.phase();
  effect.update( 0.1 );
  EXPECT_GT( effect.phase(), before );
}

TEST( BatteryPulseEffectTest, RenderBlendsRed )
{
  BatteryPulseEffect effect;
  std::array<uint16_t, 8> cells1 = { 3600, 4000, 4000, 4000, 4000, 4000, 4000, 4000 };
  std::array<uint16_t, 8> cells2 = { 4000, 4000, 4000, 3600, 4000, 4000, 4000, 4000 };
  effect.updateBatteryState( cells1, cells2 );

  // Advance to a point where pulse > 0
  effect.update( 0.25 / BatteryPulseEffect::PULSE_FREQUENCY_HZ );

  std::vector<Color> pixels( LED_COUNT, Color( 0, 80, 255 ) );
  effect.render( pixels );

  for ( const auto &p : pixels ) EXPECT_GT( p.r, 0 ); // red component increased
}

// ============================================================================
// PowerSupplyEffect tests
// ============================================================================

TEST( PowerSupplyEffectTest, InactiveByDefault )
{
  PowerSupplyEffect effect( LED_COUNT );
  EXPECT_FALSE( effect.isActive() );
}

TEST( PowerSupplyEffectTest, ActiveWhenOnPowerSupply )
{
  PowerSupplyEffect effect( LED_COUNT );
  effect.setOnPowerSupply( true );
  EXPECT_TRUE( effect.isActive() );
}

TEST( PowerSupplyEffectTest, MultipleLedsPerGroup )
{
  PowerSupplyEffect effect( LED_COUNT );
  effect.setOnPowerSupply( true );

  std::vector<Color> pixels( LED_COUNT, Color( 0, 0, 0 ) );
  effect.render( pixels );

  int green_count = 0;
  for ( const auto &p : pixels ) {
    if ( p.g > 10 )
      green_count++;
  }
  // 4 groups of 4 LEDs, each group blended across up to 5 LEDs = up to 20
  EXPECT_GE( green_count, PowerSupplyEffect::NUM_LED_GROUPS * PowerSupplyEffect::NUM_LEDS_PER_GROUP );
  EXPECT_LE( green_count,
             PowerSupplyEffect::NUM_LED_GROUPS * ( PowerSupplyEffect::NUM_LEDS_PER_GROUP + 1 ) );
}

TEST( PowerSupplyEffectTest, PositionAdvances )
{
  PowerSupplyEffect effect( LED_COUNT );
  effect.setOnPowerSupply( true );
  double before = effect.position();
  effect.update( 0.1 );
  EXPECT_GT( effect.position(), before );
}

TEST( PowerSupplyEffectTest, OverlaysOnBase )
{
  PowerSupplyEffect effect( LED_COUNT );
  effect.setOnPowerSupply( true );

  std::vector<Color> pixels( LED_COUNT, Color( 0, 80, 255 ) );
  effect.render( pixels );

  // Most pixels should still be blue (unchanged)
  int blue_count = 0;
  for ( const auto &p : pixels ) {
    if ( p == Color( 0, 80, 255 ) )
      blue_count++;
  }
  EXPECT_GE( blue_count, LED_COUNT - PowerSupplyEffect::NUM_LED_GROUPS *
                                         ( PowerSupplyEffect::NUM_LEDS_PER_GROUP + 1 ) );
}

// ============================================================================
// SpotLightEffect tests
// ============================================================================

TEST( SpotLightEffectTest, InactiveByDefault )
{
  SpotLightEffect effect( LED_COUNT );
  EXPECT_FALSE( effect.isActive() );
}

TEST( SpotLightEffectTest, EnableDisable )
{
  SpotLightEffect effect( LED_COUNT );
  effect.setSpotLight( true, 1.0f, 0.0f, 360.0f );
  EXPECT_TRUE( effect.isActive() );
  effect.setSpotLight( false, 1.0f, 0.0f, 360.0f );
  EXPECT_FALSE( effect.isActive() );
}

TEST( SpotLightEffectTest, FullCircle )
{
  SpotLightEffect effect( LED_COUNT );
  effect.setSpotLight( true, 1.0f, 0.0f, 360.0f );

  std::vector<Color> pixels( LED_COUNT, Color( 0, 0, 0 ) );
  effect.render( pixels );

  for ( const auto &p : pixels ) EXPECT_EQ( p, Color( 255, 255, 255 ) );
}

TEST( SpotLightEffectTest, DirectionalArc )
{
  SpotLightEffect effect( LED_COUNT );
  // Direction at 0°, width 36° → ~11 LEDs
  effect.setSpotLight( true, 1.0f, 0.0f, 36.0f );

  std::vector<Color> pixels( LED_COUNT, Color( 0, 0, 0 ) );
  effect.render( pixels );

  int lit = 0;
  for ( const auto &p : pixels ) {
    if ( p.r > 0 || p.g > 0 || p.b > 0 )
      lit++;
  }
  EXPECT_GE( lit, 8 );
  EXPECT_LE( lit, 16 );

  // Opposite side should be black
  EXPECT_EQ( pixels[55], Color( 0, 0, 0 ) );
}

TEST( SpotLightEffectTest, DirectionalArcWrapsAround )
{
  SpotLightEffect effect( LED_COUNT );
  // Direction at 350°, width 40° → wraps around 0
  effect.setSpotLight( true, 1.0f, 350.0f, 40.0f );

  std::vector<Color> pixels( LED_COUNT, Color( 0, 0, 0 ) );
  effect.render( pixels );

  EXPECT_GT( pixels[0].r, 0 );               // near 0° is lit
  EXPECT_EQ( pixels[55], Color( 0, 0, 0 ) ); // opposite side is dark
}

TEST( SpotLightEffectTest, BrightnessScaling )
{
  SpotLightEffect effect( LED_COUNT );
  effect.setSpotLight( true, 0.5f, 0.0f, 360.0f );

  std::vector<Color> pixels( LED_COUNT, Color( 0, 0, 0 ) );
  effect.render( pixels );

  for ( const auto &p : pixels ) {
    EXPECT_GT( p.r, 100 );
    EXPECT_LT( p.r, 200 );
  }
}

TEST( SpotLightEffectTest, OverlayPreservesBase )
{
  SpotLightEffect effect( LED_COUNT );
  effect.setSpotLight( true, 1.0f, 90.0f, 36.0f );

  std::vector<Color> pixels( LED_COUNT, Color( 0, 80, 255 ) );
  effect.render( pixels );

  // LEDs far from 90° should still be blue
  EXPECT_EQ( pixels[0], Color( 0, 80, 255 ) );
}

// ============================================================================
// RainbowLoadingEffect tests
// ============================================================================

TEST( RainbowLoadingEffectTest, ActiveByDefault )
{
  RainbowLoadingEffect effect( LED_COUNT );
  EXPECT_TRUE( effect.isActive() );
}

TEST( RainbowLoadingEffectTest, Deactivate )
{
  RainbowLoadingEffect effect( LED_COUNT );
  effect.deactivate();
  EXPECT_FALSE( effect.isActive() );
}

TEST( RainbowLoadingEffectTest, RenderProducesColorfulPixels )
{
  RainbowLoadingEffect effect( LED_COUNT );

  std::vector<Color> pixels( LED_COUNT, Color( 0, 0, 0 ) );
  effect.render( pixels );

  // Should have a variety of colors (full hue spectrum)
  bool has_red = false, has_green = false, has_blue = false;
  uint8_t threshold = static_cast<uint8_t>( 200 * RainbowLoadingEffect::BRIGHTNESS );
  for ( const auto &p : pixels ) {
    if ( p.r > threshold )
      has_red = true;
    if ( p.g > threshold )
      has_green = true;
    if ( p.b > threshold )
      has_blue = true;
  }
  EXPECT_TRUE( has_red );
  EXPECT_TRUE( has_green );
  EXPECT_TRUE( has_blue );
}

TEST( RainbowLoadingEffectTest, OffsetAdvances )
{
  RainbowLoadingEffect effect( LED_COUNT );
  double before = effect.offset();
  effect.update( 0.1 );
  EXPECT_GT( effect.offset(), before );
}

TEST( RainbowLoadingEffectTest, InactiveRenderDoesNothing )
{
  RainbowLoadingEffect effect( LED_COUNT );
  effect.deactivate();

  std::vector<Color> pixels( LED_COUNT, Color( 42, 42, 42 ) );
  effect.render( pixels );

  for ( const auto &p : pixels ) EXPECT_EQ( p, Color( 42, 42, 42 ) );
}

// ============================================================================
// BatteryConnectionEffect Tests
// ============================================================================

TEST( BatteryConnectionEffectTest, InactiveByDefault )
{
  BatteryConnectionEffect effect( LED_COUNT );
  EXPECT_FALSE( effect.isActive() );
}

TEST( BatteryConnectionEffectTest, TriggersOnConnection )
{
  BatteryConnectionEffect effect( LED_COUNT );
  std::array<uint16_t, 8> low = { 3500, 3500, 3500, 3500, 3500, 3500, 3500, 3500 };
  std::array<uint16_t, 8> disconnected = { 0, 0, 0, 0, 0, 0, 0, 0 };

  effect.updateBatteryState( low, disconnected );
  EXPECT_TRUE( effect.isActive() ); // Battery 1 connected
}

TEST( BatteryConnectionEffectTest, AnimationProgress )
{
  BatteryConnectionEffect effect( LED_COUNT );
  std::array<uint16_t, 8> ok = { 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000 };
  std::array<uint16_t, 8> disconnected = { 0, 0, 0, 0, 0, 0, 0, 0 };

  effect.updateBatteryState( ok, disconnected );

  std::vector<Color> pixels( LED_COUNT, Color( 0, 0, 0 ) );
  effect.render( pixels );

  // At t=0, progress=0, no LEDs should be green
  int green_count = 0;
  for ( const auto &p : pixels )
    if ( p.g > 0 )
      green_count++;
  EXPECT_EQ( green_count, 0 );

  // At t=0.5s, half the animation (which is 1s total)
  effect.update( 0.5 );
  pixels.assign( LED_COUNT, Color( 0, 0, 0 ) );
  effect.render( pixels );

  green_count = 0;
  for ( const auto &p : pixels )
    if ( p.g > 0 )
      green_count++;
  // Each battery side is 110/2 = 55 LEDs. At 50%, it should be around 27-28 LEDs.
  EXPECT_NEAR( green_count, 27, 2 );

  // At t=0.99s, almost full animation
  effect.update( 0.49 );
  pixels.assign( LED_COUNT, Color( 0, 0, 0 ) );
  effect.render( pixels );

  green_count = 0;
  for ( const auto &p : pixels )
    if ( p.g > 0 )
      green_count++;
  EXPECT_NEAR( green_count, 55, 2 );

  // At t=1.01s, animation finished, should be inactive
  effect.update( 0.02 );
  EXPECT_FALSE( effect.isActive() );
}

TEST( BatteryConnectionEffectTest, DisconnectionAnimation )
{
  BatteryConnectionEffect effect( LED_COUNT );
  std::array<uint16_t, 8> ok = { 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000 };
  std::array<uint16_t, 8> disconnected = { 0, 0, 0, 0, 0, 0, 0, 0 };

  // Connect
  effect.updateBatteryState( ok, disconnected );
  effect.update( 1.01 ); // Finish connection animation
  EXPECT_FALSE( effect.isActive() );

  // Disconnect
  effect.updateBatteryState( disconnected, disconnected );
  EXPECT_TRUE( effect.isActive() );

  // 0.2s quick fill to 100%
  effect.update( 0.1 );
  std::vector<Color> pixels( LED_COUNT, Color( 0, 0, 0 ) );
  effect.render( pixels );
  int green_count = 0;
  for ( const auto &p : pixels )
    if ( p.g > 0 )
      green_count++;
  EXPECT_NEAR( green_count, 27, 2 );

  effect.update( 0.11 ); // Finish quick fill, enter shrink
  // Now it's in shrink state (0.8s)
  effect.update( 0.4 ); // 50% of shrink
  pixels.assign( LED_COUNT, Color( 0, 0, 0 ) );
  effect.render( pixels );
  green_count = 0;
  for ( const auto &p : pixels )
    if ( p.g > 0 )
      green_count++;
  EXPECT_NEAR( green_count, 27, 2 );

  effect.update( 0.41 ); // Finish shrink
  EXPECT_FALSE( effect.isActive() );
}

TEST( BatteryConnectionEffectTest, JointAngleRotation )
{
  // This test now only verifies that BatteryConnectionEffect renders at fixed indices
  // (27/28 for +90 deg), because global rotation is handled by the controller.
  BatteryConnectionEffect effect( LED_COUNT );
  std::array<uint16_t, 8> ok = { 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000 };
  std::array<uint16_t, 8> disconnected = { 0, 0, 0, 0, 0, 0, 0, 0 };

  effect.updateBatteryState( ok, disconnected );
  effect.update( 0.99 );

  std::vector<Color> pixels( LED_COUNT, Color( 0, 0, 0 ) );
  effect.render( pixels );
  EXPECT_EQ( pixels[27], Color( 0, 255, 0 ) ); // Center of side 1 (+90 deg)
  EXPECT_EQ( pixels[28], Color( 0, 255, 0 ) );

  // Battery 2 is at -90 deg (270 deg) -> index 110 * 0.75 = 82.5
  BatteryConnectionEffect effect2( LED_COUNT );
  effect2.updateBatteryState( disconnected, ok );
  effect2.update( 0.99 );
  pixels.assign( LED_COUNT, Color( 0, 0, 0 ) );
  effect2.render( pixels );
  EXPECT_EQ( pixels[82], Color( 0, 255, 0 ) );
  EXPECT_EQ( pixels[83], Color( 0, 255, 0 ) );
}

// ============================================================================
// LedRingController (render pipeline) tests
// ============================================================================

TEST( LedRingControllerTest, TickSendsFrame )
{
  auto transport = std::make_shared<MockTransport>();
  transport->open();

  LedRingController controller( LED_COUNT, transport );
  controller.tick( 0.033 );

  EXPECT_EQ( transport->frameCount(), 1u );
  EXPECT_EQ( transport->lastFrame().size(), LED_COUNT );
}

TEST( LedRingControllerTest, Rotation )
{
  auto transport = std::make_shared<MockTransport>();
  transport->open();

  LedRingController controller( LED_COUNT, transport );
  // Create a dummy effect that sets one pixel
  struct DummyEffect : public LedEffect {
    bool isActive() const override { return true; }
    void update( double ) override { }
    void render( std::vector<Color> &pixels ) override { pixels[0] = Color( 255, 0, 0 ); }
  };
  controller.addEffect( std::make_shared<DummyEffect>() );

  // No rotation -> pixel 0 is red
  controller.setRotation( 0.0 );
  controller.tick( 0.0 );
  EXPECT_EQ( transport->lastFrame()[0], Color( 255, 0, 0 ) );

  // Rotate by exactly 28 indices CCW.
  // rotation = 2 * PI * 28 / 110
  double rot28 = 2.0 * M_PI * 28.0 / static_cast<double>( LED_COUNT );
  controller.setRotation( rot28 );
  controller.tick( 0.0 );
  // index_shift = round(-28) = -28. 0 + (-28) = -28 -> 82.
  EXPECT_EQ( transport->lastFrame()[82], Color( 255, 0, 0 ) );

  // Rotate CW by 10 indices.
  double rot10cw = -2.0 * M_PI * 10.0 / static_cast<double>( LED_COUNT );
  controller.setRotation( rot10cw );
  controller.tick( 0.0 );
  // index_shift = round(10) = 10. 0 + 10 = 10.
  EXPECT_EQ( transport->lastFrame()[10], Color( 255, 0, 0 ) );
}

TEST( LedRingControllerTest, BlackWithNoEffects )
{
  auto transport = std::make_shared<MockTransport>();
  transport->open();

  LedRingController controller( LED_COUNT, transport );
  controller.tick( 0.033 );

  for ( const auto &p : transport->lastFrame() ) EXPECT_EQ( p, Color( 0, 0, 0 ) );
}

TEST( LedRingControllerTest, SingleFillEffect )
{
  auto transport = std::make_shared<MockTransport>();
  transport->open();

  auto mode = std::make_shared<OperatingModeEffect>();
  mode->setMode( "autonomous" );

  LedRingController controller( LED_COUNT, transport );
  controller.addEffect( mode );
  controller.tick( 0.033 );

  for ( const auto &p : transport->lastFrame() )
    EXPECT_EQ( p, Color( 0, 80, 255 ).scaled( OperatingModeEffect::BRIGHTNESS ) );
}

TEST( LedRingControllerTest, BlendEffectOnTop )
{
  auto transport = std::make_shared<MockTransport>();
  transport->open();

  auto mode = std::make_shared<OperatingModeEffect>();
  mode->setMode( "autonomous" );

  auto light = std::make_shared<SpotLightEffect>( LED_COUNT );
  light->setSpotLight( true, 1.0f, 0.0f, 360.0f );

  LedRingController controller( LED_COUNT, transport );
  controller.addEffect( mode );
  controller.addEffect( light );
  controller.tick( 0.033 );

  // Full white light on blue → all white
  for ( const auto &p : transport->lastFrame() ) EXPECT_EQ( p, Color( 255, 255, 255 ) );
}

TEST( LedRingControllerTest, GlobalBrightness )
{
  auto transport = std::make_shared<MockTransport>();
  transport->open();

  auto mode = std::make_shared<OperatingModeEffect>();
  mode->setMode( "safe" );

  LedRingController controller( LED_COUNT, transport );
  controller.addEffect( mode );
  controller.setBrightness( 0.5f );
  controller.tick( 0.033 );

  for ( const auto &p : transport->lastFrame() ) {
    EXPECT_EQ( p.r, 0 );
    EXPECT_EQ( p.g, static_cast<uint8_t>( 255 * OperatingModeEffect::BRIGHTNESS * 0.5f ) );
    EXPECT_EQ( p.b, 0 );
  }
}

TEST( LedRingControllerTest, InactiveEffectDoesNotAffect )
{
  auto transport = std::make_shared<MockTransport>();
  transport->open();

  auto mode = std::make_shared<OperatingModeEffect>();
  mode->setMode( "safe" );

  auto battery = std::make_shared<BatteryPulseEffect>();
  // Battery not low → inactive

  LedRingController controller( LED_COUNT, transport );
  controller.addEffect( mode );
  controller.addEffect( battery );
  controller.tick( 0.033 );

  for ( const auto &p : transport->lastFrame() )
    EXPECT_EQ( p, Color( 0, 255, 0 ).scaled( OperatingModeEffect::BRIGHTNESS ) );
}

TEST( LedRingControllerTest, LaterFillOverwritesEarlierFill )
{
  auto transport = std::make_shared<MockTransport>();
  transport->open();

  auto rainbow = std::make_shared<RainbowLoadingEffect>( LED_COUNT );
  auto mode = std::make_shared<OperatingModeEffect>();
  mode->setMode( "safe" );

  LedRingController controller( LED_COUNT, transport );
  controller.addEffect( rainbow ); // rendered first
  controller.addEffect( mode );    // overwrites rainbow

  controller.tick( 0.033 );

  // Operating mode (green) should have overwritten the rainbow
  for ( const auto &p : transport->lastFrame() )
    EXPECT_EQ( p, Color( 0, 255, 0 ).scaled( OperatingModeEffect::BRIGHTNESS ) );
}

TEST( LedRingControllerTest, RainbowShowsWhenNoModeSet )
{
  auto transport = std::make_shared<MockTransport>();
  transport->open();

  auto rainbow = std::make_shared<RainbowLoadingEffect>( LED_COUNT );
  auto mode = std::make_shared<OperatingModeEffect>();
  // Mode not set → inactive, rainbow is still active

  LedRingController controller( LED_COUNT, transport );
  controller.addEffect( rainbow );
  controller.addEffect( mode );
  controller.tick( 0.033 );

  // Should see rainbow colors (not all black)
  bool has_color = false;
  for ( const auto &p : transport->lastFrame() ) {
    if ( p.r > 0 || p.g > 0 || p.b > 0 )
      has_color = true;
  }
  EXPECT_TRUE( has_color );
}

TEST( LedRingControllerTest, MultipleBlendEffects )
{
  auto transport = std::make_shared<MockTransport>();
  transport->open();

  auto mode = std::make_shared<OperatingModeEffect>();
  mode->setMode( "autonomous" );

  auto power = std::make_shared<PowerSupplyEffect>( LED_COUNT );
  power->setOnPowerSupply( true );

  auto light = std::make_shared<SpotLightEffect>( LED_COUNT );
  light->setSpotLight( true, 0.5f, 180.0f, 36.0f );

  LedRingController controller( LED_COUNT, transport );
  controller.addEffect( mode );
  controller.addEffect( power );
  controller.addEffect( light );
  controller.tick( 0.033 );

  const auto &frame = transport->lastFrame();
  bool has_blue = false, has_modified = false;
  for ( const auto &p : frame ) {
    if ( p == Color( 0, 80, 255 ).scaled( OperatingModeEffect::BRIGHTNESS ) )
      has_blue = true;
    if ( p != Color( 0, 80, 255 ).scaled( OperatingModeEffect::BRIGHTNESS ) )
      has_modified = true;
  }
  EXPECT_TRUE( has_blue );     // Some pixels still show base
  EXPECT_TRUE( has_modified ); // Some pixels modified by overlays
}

// ============================================================================
// Integration: full pipeline
// ============================================================================

TEST( IntegrationTest, FullPipelineLifecycle )
{
  auto transport = std::make_shared<MockTransport>();
  transport->open();

  auto rainbow = std::make_shared<RainbowLoadingEffect>( LED_COUNT );
  auto mode = std::make_shared<OperatingModeEffect>();
  auto battery = std::make_shared<BatteryPulseEffect>();
  auto power = std::make_shared<PowerSupplyEffect>( LED_COUNT );
  auto light = std::make_shared<SpotLightEffect>( LED_COUNT );
  auto conn = std::make_shared<BatteryConnectionEffect>( LED_COUNT );

  LedRingController controller( LED_COUNT, transport );
  controller.addEffect( rainbow );
  controller.addEffect( mode );
  controller.addEffect( power );
  controller.addEffect( battery );
  controller.addEffect( light );
  controller.addEffect( conn );

  // Phase 1: No mode set → rainbow visible
  controller.tick( 0.033 );
  bool has_color = false;
  for ( const auto &p : transport->lastFrame() ) {
    if ( p.r > 0 || p.g > 0 || p.b > 0 )
      has_color = true;
  }
  EXPECT_TRUE( has_color );

  // Phase 2: Set mode → operating mode overwrites rainbow
  mode->setMode( "autonomous" );
  rainbow->deactivate();
  controller.tick( 0.033 );
  for ( const auto &p : transport->lastFrame() )
    EXPECT_EQ( p, Color( 0, 80, 255 ).scaled( OperatingModeEffect::BRIGHTNESS ) );

  // Phase 3: Enable low battery → red pulse
  std::array<uint16_t, 8> low = { 3500, 4000, 4000, 4000, 4000, 4000, 4000, 4000 };
  battery->updateBatteryState( low, low );
  controller.tick( 0.25 / BatteryPulseEffect::PULSE_FREQUENCY_HZ ); // advance to get nonzero pulse

  bool has_red = false;
  for ( const auto &p : transport->lastFrame() ) {
    if ( p.r > 0 )
      has_red = true;
  }
  EXPECT_TRUE( has_red );

  // Phase 4: Switch to power supply, clear low battery
  std::array<uint16_t, 8> ok = { 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000 };
  battery->updateBatteryState( ok, ok );
  power->setOnPowerSupply( true );
  controller.tick( 0.033 );

  int green_count = 0;
  for ( size_t i = 0; i < transport->lastFrame().size(); ++i ) {
    const auto &p = transport->lastFrame()[i];
    if ( p.r > 0 || p.g > 0 || p.b > 0 ) {
      // std::printf("LED[%zu]: %d, %d, %d\n", i, p.r, p.g, p.b);
    }
    if ( p.g > 10 )
      green_count++;
  }
  EXPECT_GE( green_count, 4 );

  // Phase 5: Battery connection animation
  std::array<uint16_t, 8> disconnected = { 0, 0, 0, 0, 0, 0, 0, 0 };
  conn->updateBatteryState( ok, disconnected );
  controller.tick( 0.5 );
  bool has_green = false;
  for ( const auto &p : transport->lastFrame() )
    if ( p.g > 200 )
      has_green = true;
  EXPECT_TRUE( has_green );
}

int main( int argc, char **argv )
{
  ::testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
