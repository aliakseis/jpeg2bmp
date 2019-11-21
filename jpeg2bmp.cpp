// jpeg2bmp.cpp : Defines the entry point for the console application.
//

#include "kdtree.h"

#include <boost/gil/extension/io/jpeg.hpp>

#include <iostream>

#include <array>
#include <set>
#include <vector>
#include <random>
#include <limits.h>
#include <float.h>
#include <math.h>

#include <Windows.h>

const WORD DIB_HEADER_MARKER = ('M' << 8) | 'B';

template<typename T1, typename T2>
bool CreateBitmap(LPCSTR lpFileName, int g_nWidth, int g_nHeight, T1 paletteFunc, T2 pixelsFunc)
{
    HANDLE hFile = ::CreateFileA(
        lpFileName,					    // pointer to name of the file
        GENERIC_READ | GENERIC_WRITE,   // access (read-write) mode
        0,							    // share mode 
        NULL,						    // pointer to security attributes 
        CREATE_ALWAYS,				    // how to create 
        FILE_ATTRIBUTE_NORMAL,		    // file attributes 
        NULL						    // handle to file with attributes to copy
    );
    if (INVALID_HANDLE_VALUE == hFile)
        return false;

    g_nWidth = (g_nWidth + 3) & ~3; // Should be normalized

    enum {
        DataOffset = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER)
        + sizeof(RGBQUAD) * 256
    };
    int nTotalSize = DataOffset + g_nWidth * g_nHeight;

    HANDLE hFileMapping = ::CreateFileMapping(
        hFile,		    // handle to file to map 
        NULL,		    // optional security attributes 
        PAGE_READWRITE, // protection for mapping object 
        0,			    // high-order 32 bits of object size 
        nTotalSize,	    // low-order 32 bits of object size 
        NULL		    // name of file-mapping object 
    );
    if (NULL == hFileMapping)
    {
        ::CloseHandle(hFile);
        return false;
    }
    LPVOID pBuf = ::MapViewOfFile(hFileMapping, FILE_MAP_WRITE, 0, 0, nTotalSize);
    if (NULL == pBuf)
    {
        ::CloseHandle(hFile);
        ::CloseHandle(hFileMapping);
        return false;
    }

    BITMAPFILEHEADER* pBitmapFileHeader = (BITMAPFILEHEADER*)pBuf;
    pBitmapFileHeader->bfType = DIB_HEADER_MARKER;
    pBitmapFileHeader->bfSize = nTotalSize;
    pBitmapFileHeader->bfReserved1 = 0;
    pBitmapFileHeader->bfReserved2 = 0;
    pBitmapFileHeader->bfOffBits = DataOffset;

    BITMAPINFOHEADER* pBitmapInfoHeader
        = (BITMAPINFOHEADER*)(((BITMAPFILEHEADER*)pBuf) + 1);
    memset(pBitmapInfoHeader, 0, sizeof(BITMAPINFOHEADER));

    pBitmapInfoHeader->biSize = sizeof(BITMAPINFOHEADER);
    pBitmapInfoHeader->biWidth = g_nWidth;
    pBitmapInfoHeader->biHeight = g_nHeight;
    pBitmapInfoHeader->biPlanes = 1;
    pBitmapInfoHeader->biBitCount = 8;
    pBitmapInfoHeader->biCompression = BI_RGB;

    RGBQUAD* pRGBQUAD = (RGBQUAD*)(pBitmapInfoHeader + 1);

    paletteFunc(pRGBQUAD);

    BYTE* g_pBytes = ((BYTE*)pBuf) + DataOffset;

    pixelsFunc(g_pBytes, g_nWidth);

    ::UnmapViewOfFile(pBuf);
    ::CloseHandle(hFile);
    ::CloseHandle(hFileMapping);

    return true;
}

// C:\boost_1_63_0\libs\gil\example\test.jpg

namespace gil = boost::gil;

typedef std::array<unsigned char, 3> pixel;

std::set<pixel> getInitialPoints(const gil::rgb8_image_t& img, int seed)
{
    const auto& view = const_view(img);

    const auto size = view.size();

    std::vector<int> indices(size);

    std::default_random_engine dre(seed);
    for (int i = 0; i < size; ++i)
    {
        std::uniform_int_distribution<int> di(0, i);
        const int j = di(dre);
        if (i != j)
            indices[i] = indices[j];
        indices[j] = i;
    }

    std::set<pixel> result;
    for (const auto idx : indices)
    {
        gil::rgb8_pixel_t px = *view.at(idx);
        if (result.insert({ px[0], px[1], px[2] }).second
            && result.size() == 256)
        {
            break;
        }
    }

    return result;
}

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: jpeg2bmp input_file output_file\n";
        return 1;
    }

    gil::rgb8_image_t img;
    gil::read_image(argv[1], img, gil::jpeg_tag());

    enum { NUM_ATTEMPTS = 10 };

    ObjectInfos storedInfos[NUM_ATTEMPTS];
    kdnode *storedRoots[NUM_ATTEMPTS];
    double storedErrors[NUM_ATTEMPTS];

    const auto& view = const_view(img);

#pragma omp parallel for
    for (int attempt = 0; attempt < NUM_ATTEMPTS; ++attempt)
    {
        ObjectInfos infos;

        for (const auto& point : getInitialPoints(img, attempt))
        {
            infos.push_back({ point });
        }

        double error = DBL_MAX;

        for (;;)
        {
            std::vector<ObjectInfo*> infoPtrs;
            infoPtrs.reserve(infos.size());

            for (auto it = infos.begin(); it != infos.end(); ++it)
            {
                infoPtrs.push_back(&*it);
            }

            auto root = insert(infoPtrs.begin(), infoPtrs.end(), nullptr, 0);

            for (gil::rgb8_pixel_t px : view)
            {
                SearchResults result;
                bool flags[DIM * 2]{};
                pixel data{ px[0], px[1], px[2] };
                kd_nearest_i_nearer_subtree(root, data.data(), result, flags, 0);
                auto& a = result.node()->data;
                std::transform(a.begin(), a.end(), data.begin(), a.begin(), std::plus<DistanceType>());
                auto& sq_a = result.node()->sq_data;
                std::transform(sq_a.begin(), sq_a.end(), data.begin(), sq_a.begin(), 
                    [](auto left, auto right) { return left + right * right; });
                ++result.node()->count;
            }

            double newError = 0;
            for (const auto& item : infos)
            {
                const double count = item.count;
                const auto delta 
                        = item.sq_data[0] - item.data[0] * (item.data[0] / count)
                        + item.sq_data[1] - item.data[1] * (item.data[1] / count)
                        + item.sq_data[2] - item.data[2] * (item.data[2] / count);
                newError += delta;
            }


            if (newError >= error)
                break;

            error = newError;

            ObjectInfos newInfos;

            for (const auto& item : infos)
            {
                pixel px{
                    (item.data[0] + item.count / 2) / item.count,
                    (item.data[1] + item.count / 2) / item.count,
                    (item.data[2] + item.count / 2) / item.count };
                newInfos.push_back({ px });
            }

            storedRoots[attempt] = root;
            storedErrors[attempt] = error;
            storedInfos[attempt] = std::move(infos);
            infos = std::move(newInfos);
        }

        //std::cout << attempt << ' ' << error << '\n';
    }

    int optimalIdx = 0;
    double minError = storedErrors[0];
    for (int i = 1; i < NUM_ATTEMPTS; ++i)
    {
        if (storedErrors[i] < minError)
        {
            minError = storedErrors[i];
            optimalIdx = i;
        }
    }

    auto root = storedRoots[optimalIdx];
    const ObjectInfos& infos = storedInfos[optimalIdx];

    const auto width = img.width();
    const auto height = img.height();

    CreateBitmap(argv[2], width, height,
        [&infos](RGBQUAD* pRGBQUAD) {
            for (int i = 0; i < 256; i++, pRGBQUAD++)
            {
                pRGBQUAD->rgbRed = infos[i].pos[0];
                pRGBQUAD->rgbGreen = infos[i].pos[1];
                pRGBQUAD->rgbBlue = infos[i].pos[2];
                pRGBQUAD->rgbReserved = 0;
            }
        },
        [&view, root, &infos](BYTE* pBytes, int pitch) {
            const auto width = view.width();
            const auto height = view.height();
            for (int i = 0; i < height; ++i)
                for (int j = 0; j < width; ++j)
                {
                    gil::rgb8_pixel_t px = *view.at(j, height - 1 - i);

                    SearchResults result;
                    bool flags[DIM * 2]{};
                    pixel data{ px[0], px[1], px[2] };
                    kd_nearest_i_nearer_subtree(root, data.data(), result, flags, 0);
                    auto v = result.node() - infos.data();
                    pBytes[i * pitch + j] = (BYTE)v;
                }
        });

    return 0;
}
