# Queue Policy (Week 8)

## Goal

This policy defines how the current pipeline handles overload conditions while prioritizing real-time responsiveness.

The main objective is to prevent unbounded queue growth, limit latency increase, and keep the system processing fresh data instead of stale data.

## Policy Summary

* Use a **bounded queue** for incoming vehicle messages.
* When the queue is full, apply **drop-oldest**.
* Track queue pressure using **warning** and **critical watermarks**.
* Record runtime observability metrics such as:

  * push count
  * pop count
  * drop count
  * warning count
  * critical count
  * process time

## Why Bounded Queue

If input arrives faster than processing, an unbounded queue can keep growing indefinitely.
That leads to higher latency, stale data accumulation, and unstable runtime behavior.

A bounded queue limits how much backlog the system can accumulate.

## Why Drop-Oldest

This implementation prioritizes **fresh data** over lossless processing.

When the queue is full, dropping the oldest message is more suitable for real-time systems than dropping the newest one.
If the newest message is discarded, the system keeps processing outdated data and falls further behind the current state.

## Why Watermarks

Watermarks provide early visibility before the queue becomes fully saturated.

* **Warning watermark** indicates that backlog is increasing.
* **Critical watermark** indicates that the queue is close to saturation.

This helps observe overload progression before repeated drops occur.

## Observability Metrics

To understand overload behavior, the system records:

* how many messages were pushed into the queue
* how many messages were popped by the worker
* how many messages were dropped
* how often warning and critical watermarks were hit
* how processing time changed under load

These metrics are used to explain whether the system is keeping up with input or falling behind.

## Current Scope

At this stage, overload is reproduced by slowing down the consumer side of the pipeline.
This is a simplified backpressure validation step before extending the system to more realistic bottlenecks such as save, encode, or disk-related delays.

## Design Priority

The current design favors **bounded latency and fresher data** over processing every message without loss.
