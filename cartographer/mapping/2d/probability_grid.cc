/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "cartographer/mapping/2d/probability_grid.h"

#include <limits>

#include "absl/memory/memory.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/submaps.h"

namespace cartographer {
namespace mapping {

ProbabilityGrid::ProbabilityGrid(const MapLimits& limits,
                                 ValueConversionTables* conversion_tables)
    : Grid2D(limits, kMinCorrespondenceCost, kMaxCorrespondenceCost,
             conversion_tables),
      conversion_tables_(conversion_tables) {}

ProbabilityGrid::ProbabilityGrid(const proto::Grid2D& proto,
                                 ValueConversionTables* conversion_tables)
    : Grid2D(proto, conversion_tables), conversion_tables_(conversion_tables) {
  CHECK(proto.has_probability_grid_2d());
}

// Sets the probability of the cell at 'cell_index' to the given
// 'probability'. Only allowed if the cell was unknown before.
void ProbabilityGrid::SetProbability(const Eigen::Array2i& cell_index,
                                     const float probability) {
  uint16& cell =
      (*mutable_correspondence_cost_cells())[ToFlatIndex(cell_index)];
  CHECK_EQ(cell, kUnknownProbabilityValue);
  /*
    占据概率:probability
    占据概率转换成空闲概率: ProbabilityToCorrespondenceCost(probability)
    空闲概率转换成Value : cell = CorrespondenceCostToValue()
  */ 
  cell =
      CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(probability));
  
  // 已知栅格扩展
  mutable_known_cells_box()->extend(cell_index.matrix());
}

// Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
// to the probability of the cell at 'cell_index' if the cell has not already
// been updated. Multiple updates of the same cell will be ignored until
// FinishUpdate() is called. Returns true if the cell was updated.
//
// If this is the first call to ApplyOdds() for the specified cell, its value
// will be set to probability corresponding to 'odds'.

// 查表更新栅格概率
bool ProbabilityGrid::ApplyLookupTable(const Eigen::Array2i& cell_index,
                                       const std::vector<uint16>& table) {
  DCHECK_EQ(table.size(), kUpdateMarker);
  // 通过位置,找到索引
  const int flat_index = ToFlatIndex(cell_index);
  
  //mutable_correspondence_cost_cells() = correspondence_cost_cells_
  /*
      当 flat_index 是一个新点时. cell = 0
      否则, cell 等于其概率对应的 Int16(value)
  */ 
  uint16* cell = &(*mutable_correspondence_cost_cells())[flat_index];
  
  /*
    地图中的value 都小于 kUpdateMarker.
    但是hit_table_ 中的value 都大于 kUpdateMarker.
    所以每次hit_table_更新完之后,在finish时,都减去一个 kUpdateMarker
  */ 
  if (*cell >= kUpdateMarker) {
    return false;
  }

  //mutable_update_indices() = update_indices_
  /*
    将更新的点都保存在 update_indices_ 容器中,结束更新之后,统一将概率值减去 kUpdateMarker
  */ 
  mutable_update_indices()->push_back(flat_index);

  // 查表更新概率
  *cell = table[*cell];
  DCHECK_GE(*cell, kUpdateMarker);

  /*
    栅格地图中,已知单元格边界更新
  */ 
  mutable_known_cells_box()->extend(cell_index.matrix());
  return true;
}

GridType ProbabilityGrid::GetGridType() const {
  return GridType::PROBABILITY_GRID;
}

// Returns the probability of the cell with 'cell_index'.
// 返回地图点在地图中的概率
float ProbabilityGrid::GetProbability(const Eigen::Array2i& cell_index) const {
  // 如果不包含cell_index　直接返回 0.1
  if (!limits().Contains(cell_index)) return kMinProbability;

  /*
    获取空闲概率对应Value:value = correspondence_cost_cells()[ToFlatIndex(cell_index)]
        该点没有更新过概率的情况下 value = 0
    value值转换成空闲概率: p = ValueToCorrespondenceCost(value)
        value = 0时, p = 0.9
    空闲概率转换成占据概率: p' = CorrespondenceCostToProbability(p)
        value = 0时,p' = 0.1
  */ 
  return CorrespondenceCostToProbability(ValueToCorrespondenceCost(
      correspondence_cost_cells()[ToFlatIndex(cell_index)]));
}

proto::Grid2D ProbabilityGrid::ToProto() const {
  proto::Grid2D result;
  result = Grid2D::ToProto();
  result.mutable_probability_grid_2d();
  return result;
}

// 计算裁剪网格.一张子地图建图完成,调用来修建地图
std::unique_ptr<Grid2D> ProbabilityGrid::ComputeCroppedGrid() const {
  Eigen::Array2i offset;
  CellLimits cell_limits;
  /*
      offset,边界角点
      cell_limits,边界尺寸
  */ 
  ComputeCroppedLimits(&offset, &cell_limits);
  const double resolution = limits().resolution();
  // 计算offset角点,在地图中的坐标 max
  const Eigen::Vector2d max =
      limits().max() - resolution * Eigen::Vector2d(offset.y(), offset.x());
  
  // 初始化 ProbabilityGrid 局部变量
  std::unique_ptr<ProbabilityGrid> cropped_grid =
      absl::make_unique<ProbabilityGrid>(
          MapLimits(resolution, max, cell_limits), conversion_tables_);
  // 对该裁剪地图进行概率赋值
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
    if (!IsKnown(xy_index + offset)) continue;
    cropped_grid->SetProbability(xy_index, GetProbability(xy_index + offset));
  }
  
  // 返回裁剪后的地图
  return std::unique_ptr<Grid2D>(cropped_grid.release());
}

// 绘制地图
bool ProbabilityGrid::DrawToSubmapTexture(
    proto::SubmapQuery::Response::SubmapTexture* const texture,
    transform::Rigid3d local_pose) const {
  Eigen::Array2i offset;
  CellLimits cell_limits;
  ComputeCroppedLimits(&offset, &cell_limits);

  // LOG(ERROR)<<"\n debug 5";

  std::string cells;
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
    if (!IsKnown(xy_index + offset)) {
      cells.push_back(0 /* unknown log odds value */);
      cells.push_back(0 /* alpha */);
      continue;
    }
    // We would like to add 'delta' but this is not possible using a value and
    // alpha. We use premultiplied alpha, so when 'delta' is positive we can
    // add it by setting 'alpha' to zero. If it is negative, we set 'value' to
    // zero, and use 'alpha' to subtract. This is only correct when the pixel
    // is currently white, so walls will look too gray. This should be hard to
    // detect visually for the user, though.
    /*
        ProbabilityToLogOddsInteger,计算 log
        p = GetProbability(xy_index + offset) 当该点没有更新过概率的情况下, p=0.1
        ProbabilityToLogOddsInteger(p) = 1 (p=0.1) 其他时候取值在[1,255]
    */ 
    const int delta =
        128 - ProbabilityToLogOddsInteger(GetProbability(xy_index + offset));
    const uint8 alpha = delta > 0 ? 0 : -delta;
    const uint8 value = delta > 0 ? delta : 0;
    cells.push_back(value);
    cells.push_back((value || alpha) ? alpha : 1);
  }

  common::FastGzipString(cells, texture->mutable_cells());
  texture->set_width(cell_limits.num_x_cells);
  texture->set_height(cell_limits.num_y_cells);
  const double resolution = limits().resolution();
  texture->set_resolution(resolution);
  const double max_x = limits().max().x() - resolution * offset.y();
  const double max_y = limits().max().y() - resolution * offset.x();
  *texture->mutable_slice_pose() = transform::ToProto(
      local_pose.inverse() *
      transform::Rigid3d::Translation(Eigen::Vector3d(max_x, max_y, 0.)));

  return true;
}

}  // namespace mapping
}  // namespace cartographer
