#include <errno.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

static void usage(const char* prog) {
    fprintf(stderr,
            "Usage: %s --device /dev/spidevX.Y --speed <Hz> --mode <0-3> --frame-bytes <N> "
            "[--frames COUNT] [--output FILE]\n",
            prog);
}

static void write_all(int fd, const uint8_t* data, size_t length) {
    size_t written = 0;
    while (written < length) {
        ssize_t rc = write(fd, data + written, length - written);
        if (rc < 0) {
            perror("write");
            exit(EXIT_FAILURE);
        }
        written += (size_t)rc;
    }
}

int main(int argc, char** argv) {
    const char* device_path = NULL;
    const char* output_path = NULL;
    uint32_t speed_hz = 0;
    uint8_t mode = SPI_MODE_3;
    size_t frame_bytes = 0;
    long frames_to_capture = -1;  // negative => infinite

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--device") && i + 1 < argc) {
            device_path = argv[++i];
        } else if (!strcmp(argv[i], "--speed") && i + 1 < argc) {
            speed_hz = (uint32_t)strtoul(argv[++i], NULL, 0);
        } else if (!strcmp(argv[i], "--mode") && i + 1 < argc) {
            mode = (uint8_t)strtoul(argv[++i], NULL, 0);
        } else if (!strcmp(argv[i], "--frame-bytes") && i + 1 < argc) {
            frame_bytes = (size_t)strtoul(argv[++i], NULL, 0);
        } else if (!strcmp(argv[i], "--frames") && i + 1 < argc) {
            frames_to_capture = strtol(argv[++i], NULL, 0);
        } else if (!strcmp(argv[i], "--output") && i + 1 < argc) {
            output_path = argv[++i];
        } else {
            usage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    if (!device_path || speed_hz == 0 || frame_bytes == 0) {
        usage(argv[0]);
        return EXIT_FAILURE;
    }

    int fd = open(device_path, O_RDWR);
    if (fd < 0) {
        perror("open spidev");
        return EXIT_FAILURE;
    }

    uint8_t bits_per_word = 8;
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 || ioctl(fd, SPI_IOC_RD_MODE, &mode) < 0) {
        perror("ioctl SPI_IOC_*_MODE");
        return EXIT_FAILURE;
    }
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0 ||
        ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits_per_word) < 0) {
        perror("ioctl SPI_IOC_*_BITS_PER_WORD");
        return EXIT_FAILURE;
    }
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0 ||
        ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed_hz) < 0) {
        perror("ioctl SPI_IOC_*_MAX_SPEED_HZ");
        return EXIT_FAILURE;
    }

    uint8_t* tx_buffer = calloc(frame_bytes, sizeof(uint8_t));
    uint8_t* rx_buffer = malloc(frame_bytes);
    if (!tx_buffer || !rx_buffer) {
        fprintf(stderr, "Failed to allocate SPI buffers\n");
        return EXIT_FAILURE;
    }

    int out_fd = STDOUT_FILENO;
    if (output_path) {
        out_fd = open(output_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (out_fd < 0) {
            perror("open output");
            return EXIT_FAILURE;
        }
    }

    long frames_sent = 0;
    size_t byte_transfer = frame_bytes * 123;
    while (frames_to_capture < 0 || frames_sent < frames_to_capture) {
        struct spi_ioc_transfer transfer = {
            .tx_buf = (unsigned long)tx_buffer,
            .rx_buf = (unsigned long)rx_buffer,
            .len = byte_transfer,
            .speed_hz = speed_hz,
            .bits_per_word = bits_per_word,
            .cs_change = 0,
        };

        int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
        if (ret < 1) {
            perror("ioctl SPI_IOC_MESSAGE");
            return EXIT_FAILURE;
        }

        write_all(out_fd, rx_buffer, byte_transfer);
        frames_sent++;
    }

    return EXIT_SUCCESS;
}
