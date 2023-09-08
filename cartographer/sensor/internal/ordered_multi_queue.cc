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

#include "cartographer/sensor/internal/ordered_multi_queue.h"

#include <algorithm>
#include <sstream>
#include <vector>

#include "absl/memory/memory.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {

namespace {

// Number of items that can be queued up before we log which queues are waiting
// for data.
const int kMaxQueueSize = 500;

}  // namespace

inline std::ostream& operator<<(std::ostream& out, const QueueKey& key) {
  return out << '(' << key.trajectory_id << ", " << key.sensor_id << ')';
}

OrderedMultiQueue::OrderedMultiQueue() {}

OrderedMultiQueue::~OrderedMultiQueue() {
  for (auto& entry : queues_) {
    CHECK(entry.second.finished);
  }
}

// 为传感器数据添加，数据队列，并设置传感器数据处理方法
void OrderedMultiQueue::AddQueue(const QueueKey& queue_key, Callback callback) {
  CHECK_EQ(queues_.count(queue_key), 0);
  queues_[queue_key].callback = std::move(callback);
}

void OrderedMultiQueue::MarkQueueAsFinished(const QueueKey& queue_key) {
  auto it = queues_.find(queue_key);
  CHECK(it != queues_.end()) << "Did not find '" << queue_key << "'.";
  auto& queue = it->second;
  CHECK(!queue.finished);
  queue.finished = true;
  Dispatch();
}

// 添加传感器数据到对应索引(<轨迹和传感器>)的队列中
void OrderedMultiQueue::Add(const QueueKey& queue_key,
                            std::unique_ptr<Data> data) {
  // 查找对应　轨迹的对应传感器
  auto it = queues_.find(queue_key);
  if (it == queues_.end()) {
    LOG_EVERY_N(WARNING, 1000)
        << "Ignored data for queue: '" << queue_key << "'";
    return;
  }
  // 将传感器数据　压入到对应队列中
  it->second.queue.Push(std::move(data));

  // 调用queue队列处理方法，处理传感器数据
  Dispatch();
}

// queue_.Flush()
void OrderedMultiQueue::Flush() {
  std::vector<QueueKey> unfinished_queues;
  for (auto& entry : queues_) {
    if (!entry.second.finished) {
      unfinished_queues.push_back(entry.first);
    }
  }
  for (auto& unfinished_queue : unfinished_queues) {
    MarkQueueAsFinished(unfinished_queue);
  }
}

QueueKey OrderedMultiQueue::GetBlocker() const {
  CHECK(!queues_.empty());
  return blocker_;
}

/**
 * 通过queue队列中封装的方法来处理队列中传感器数据
 *  寻找不同传感器的最早数据,和传感器的同步时刻(共同有数据的时刻),中间差值过大,就删除掉
 *  调用queue回调函数,处理数据
 *  callback = HandleCollatedSensorData
 * **/ 
void OrderedMultiQueue::Dispatch() {
  while (true) {
    const Data* next_data = nullptr;
    Queue* next_queue = nullptr;
    QueueKey next_queue_key;
    //遍历所有的 传感器，比较所有传感器数据队列中第一个数据时间，获取最小时间的传感器数据，进行处理操作
    /**
     * 遍历不同传感器数据,找到时间最早的一个传感器数据
     * **/ 
    /*
      std::map<QueueKey, Queue> queues_;
      struct Queue {
                    common::BlockingQueue<std::unique_ptr<Data>> queue;
                    Callback callback;
                    bool finished = false;
                    };
    */ 
    //获取最早的一帧传感器数据
    for (auto it = queues_.begin(); it != queues_.end();) {
      //获取队列中最早数据
      const auto* data = it->second.queue.Peek<Data>();
      if (data == nullptr) {
        //没有传感器数据
        if (it->second.finished) {
          // 传感器数据　finished,删除　queue
          queues_.erase(it++);
          continue;
        }
        //　接收不到传感器数据，而并非是　传感器数据　已处理完的情况下：
        //  blocker_ 赋值，打印出阻塞　的传感器
        CannotMakeProgress(it->first);
        return;
      }
      // 找到最早接收到的传感器数据，放入到　next_data
      if (next_data == nullptr || data->GetTime() < next_data->GetTime()) {
        next_data = data;
        next_queue = &it->second;
        next_queue_key = it->first;
      }
      CHECK_LE(last_dispatched_time_, next_data->GetTime())
          << "Non-sorted data added to queue: '" << it->first << "'";
      ++it;
    }
    if (next_data == nullptr) {
      CHECK(queues_.empty());
      return;
    }

    // If we haven't dispatched any data for this trajectory yet, fast forward
    // all queues of this trajectory until a common start time has been reached.

    // 获取所有传感器数据　最先压入队列的数据的　最大时间
    const common::Time common_start_time =
        GetCommonStartTime(next_queue_key.trajectory_id);

    // 处理传感器数据，处理传感器数据在公共时间前后一帧范围内的数据

    // case 1: 处理的数据　大于等于　所有传感器公共时间
    if (next_data->GetTime() >= common_start_time) {
      // Happy case, we are beyond the 'common_start_time' already.
      last_dispatched_time_ = next_data->GetTime();

      // 调用queue封装的处理方法，处理传感器数据,处理一帧,就弹出一帧数据
      next_queue->callback(next_queue->queue.Pop());
    } 
    else if (next_queue->queue.Size() < 2) 
    {
      // case 2:该帧数据时间小于公共时间，并且该传感器数据只有一帧
      if (!next_queue->finished) {
        // We cannot decide whether to drop or dispatch this yet.
        CannotMakeProgress(next_queue_key);
        return;
      }
      last_dispatched_time_ = next_data->GetTime();
      next_queue->callback(next_queue->queue.Pop());
    } 
    else 
    {
      // We take a peek at the time after next data. If it also is not beyond
      // 'common_start_time' we drop 'next_data', otherwise we just found the
      // first packet to dispatch from this queue.

      // case 3:该帧数据时间小于　公共时间，其对应传感器数据数量大于等于２，若下一帧数据时间也小于公共时间，舍弃第一帧数据
      // 下一帧数据时间大于公共时间，则处理该数据
      std::unique_ptr<Data> next_data_owner = next_queue->queue.Pop();
      if (next_queue->queue.Peek<Data>()->GetTime() > common_start_time) {
        last_dispatched_time_ = next_data->GetTime();
        next_queue->callback(std::move(next_data_owner));
      }
    }
  }
}

// 打印停止处理传感器数据的　阻塞传感器
void OrderedMultiQueue::CannotMakeProgress(const QueueKey& queue_key) {
  blocker_ = queue_key;
  for (auto& entry : queues_) {
    if (entry.second.queue.Size() > kMaxQueueSize) {
      LOG_EVERY_N(WARNING, 60) << "Queue waiting for data: " << queue_key;
      return;
    }
  }
}

// 比较所有传感器最早数据，获取其最大时间
common::Time OrderedMultiQueue::GetCommonStartTime(const int trajectory_id) {
  
  /*
      std::map<int, common::Time> common_start_time_per_trajectory_;
  */ 

  auto emplace_result = common_start_time_per_trajectory_.emplace(
      trajectory_id, common::Time::min());
  common::Time& common_start_time = emplace_result.first->second;
  // LOG(WARNING)
  //   // <<"\n emplace_result = "<<emplace_result
  //   <<"\n emplace_result.first = "<<emplace_result.first
  //   <<"\n emplace_result.second = "<<emplace_result.second
  //   <<"\n common_start_time = "<<common_start_time;
  if (emplace_result.second) {
    for (auto& entry : queues_) {
      if (entry.first.trajectory_id == trajectory_id) {
        common_start_time = std::max(
            common_start_time, entry.second.queue.Peek<Data>()->GetTime());
      }
    }
    LOG(INFO) << "All sensor data for trajectory " << trajectory_id
              << " is available starting at '" << common_start_time << "'.";
  }
  return common_start_time;
}

}  // namespace sensor
}  // namespace cartographer
