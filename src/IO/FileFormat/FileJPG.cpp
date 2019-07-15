// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include <IO/ClassIO/ImageIO.h>

#include <jpeglib.h>
#include <Core/Utility/Console.h>

namespace open3d {

bool ReadImageFromJPG(const std::string &filename, Image &image) {
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    FILE *file_in;
    JSAMPARRAY buffer;

    if ((file_in = fopen(filename.c_str(), "rb")) == NULL) {
        PrintWarning("Read JPG failed: unable to open file: %s\n",
                     filename.c_str());
        return false;
    }

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);
    jpeg_stdio_src(&cinfo, file_in);
    jpeg_read_header(&cinfo, TRUE);

    // We only support two channel types: gray, and RGB.
    int num_of_channels = 3;
    int bytes_per_channel = 1;
    switch (cinfo.jpeg_color_space) {
        case JCS_RGB:
        case JCS_YCbCr:
            cinfo.out_color_space = JCS_RGB;
            cinfo.out_color_components = 3;
            num_of_channels = 3;
            break;
        case JCS_GRAYSCALE:
            cinfo.jpeg_color_space = JCS_GRAYSCALE;
            cinfo.out_color_components = 1;
            num_of_channels = 1;
            break;
        case JCS_CMYK:
        case JCS_YCCK:
        default:
            PrintWarning("Read JPG failed: color space not supported.\n");
            jpeg_destroy_decompress(&cinfo);
            fclose(file_in);
            return false;
    }
    jpeg_start_decompress(&cinfo);
    image.PrepareImage(cinfo.output_width, cinfo.output_height, num_of_channels,
                       bytes_per_channel);
    int row_stride = cinfo.output_width * cinfo.output_components;
    buffer = (*cinfo.mem->alloc_sarray)((j_common_ptr)&cinfo, JPOOL_IMAGE,
                                        row_stride, 1);
    uint8_t *pdata = image.data_.data();
    while (cinfo.output_scanline < cinfo.output_height) {
        jpeg_read_scanlines(&cinfo, buffer, 1);
        memcpy(pdata, buffer[0], row_stride);
        pdata += row_stride;
    }
    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    fclose(file_in);
    return true;
}

bool WriteImageToJPG(const std::string &filename,
                     const Image &image,
                     int quality /* = 90*/) {
    if (image.HasData() == false) {
        PrintWarning("Write JPG failed: image has no data.\n");
        return false;
    }
    if (image.bytes_per_channel_ != 1 ||
        (image.num_of_channels_ != 1 && image.num_of_channels_ != 3)) {
        PrintWarning("Write JPG failed: unsupported image data.\n");
        return false;
    }
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    FILE *file_out;
    JSAMPROW row_pointer[1];

    if ((file_out = fopen(filename.c_str(), "wb")) == NULL) {
        PrintWarning("Write JPG failed: unable to open file: %s\n",
                     filename.c_str());
        return false;
    }

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_stdio_dest(&cinfo, file_out);
    cinfo.image_width = image.width_;
    cinfo.image_height = image.height_;
    cinfo.input_components = image.num_of_channels_;
    cinfo.in_color_space =
            (cinfo.input_components == 1 ? JCS_GRAYSCALE : JCS_RGB);
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);
    jpeg_start_compress(&cinfo, TRUE);
    int row_stride = image.width_ * image.num_of_channels_;
    const uint8_t *pdata = image.data_.data();
    std::vector<uint8_t> buffer(row_stride);
    while (cinfo.next_scanline < cinfo.image_height) {
        memcpy(buffer.data(), pdata, row_stride);
        row_pointer[0] = buffer.data();
        jpeg_write_scanlines(&cinfo, row_pointer, 1);
        pdata += row_stride;
    }
    jpeg_finish_compress(&cinfo);
    fclose(file_out);
    jpeg_destroy_compress(&cinfo);
    return true;
}

}  // namespace open3d
