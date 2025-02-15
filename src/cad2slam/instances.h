#pragma once

namespace cad2slam{
  void cad2slam_configurables() __attribute__((constructor));
  void cad2slam_serializables() __attribute__((constructor));
}