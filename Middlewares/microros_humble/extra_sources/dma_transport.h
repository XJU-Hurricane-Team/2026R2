/**
 * @file dma_transport.h
 * @author 
 * @brief 
 * @version 0.1
 * @date 2025-11-27
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef DMA_TRANSPORT_H
#define DMA_TRANSPORT_H



bool cubemx_transport_open(struct uxrCustomTransport * transport);

bool cubemx_transport_close(struct uxrCustomTransport * transport);

size_t cubemx_transport_write(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, uint8_t * err);

size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

#endif //DMA_TRANSPORT_H