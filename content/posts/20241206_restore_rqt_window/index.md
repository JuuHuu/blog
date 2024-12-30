---
title: "Restore rqt windows layout in wsl"
summary: "How to restore the windows layout in wsl when using ROS rqt "
categories: ["Post",]
tags: ["ROS2",]
#showSummary: true
date: 2025-12-06
draft: true
seriesOpened: false
# series: ["Docs"]
# series_order: 1
showPagination: false # show previous/next article at bottom
invertPagination: false
showTaxonomies: true
---

When using rqt in Windows wsl and accidentally move the panel out of main window, the panel will no longer controllable.

![](1.png)

## Solution

- Close the rqt window
- Navigate to ROS config directory

  ```console
  cd ~/.config/ros.org/
  ```

- Remove the rqt windows config file

  ```console
  rm rqt_gui.inicon
  ```

- Restart the rqt

  ```console
  rqt
  ```

  


## Reference

[https://stackoverflow.com/questions/76049607/rqt-graph-in-ros2-is-corrupted-after-maximizing-it](https://stackoverflow.com/questions/76049607/rqt-graph-in-ros2-is-corrupted-after-maximizing-it)

