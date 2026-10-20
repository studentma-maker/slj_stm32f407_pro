/**
  * @file    fw_version.h
  * @brief   下位机应用程序版本号
  * @details 每次发布给现场升级的固件都要改这里。版本号编进固件信息结构（mb_hook.c 的 g_fwuInfo），
  *          上位机从 U 盘 .bin 里读出它与下位机当前版本对比后再确认升级；升级完成后上位机
  *          读回下位机版本，与文件版本一致才算成功。主/次/修订各取 0~255。
  */
#ifndef _FW_VERSION_H
#define _FW_VERSION_H

#define FW_VERSION_MAJOR    1
#define FW_VERSION_MINOR    0
#define FW_VERSION_PATCH    0

#endif /* _FW_VERSION_H */
