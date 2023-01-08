#include "SaveLAZ.h"
#include "laszip/laszip_api.h"
bool SaveLAZ::exportLaz(const std::string& filename,
               const std::vector<Eigen::Vector3f>& pointcloud,
               const std::vector<uint16_t>& intensity){

    constexpr float scale = 0.001f;
    constexpr float scale_inv = 1.0f/scale;
    // find max
    Eigen::Vector3f max (std::numeric_limits<float>::min(),std::numeric_limits<float>::min(),std::numeric_limits<float>::min());
    Eigen::Vector3f min (std::numeric_limits<float>::max(),std::numeric_limits<float>::max(),std::numeric_limits<float>::max());
    for (auto &p: pointcloud){
        max.x() = std::max(max.x(), p.x());
        max.y() = std::max(max.y(), p.y());
        max.y() = std::max(max.z(), p.z());

        min.x() = std::min(min.x(), p.x());
        min.y() = std::min(min.y(), p.y());
        min.z() = std::min(min.z(), p.z());
    }

    // create the writer
    laszip_POINTER laszip_writer;
    if (laszip_create(&laszip_writer))
    {
        fprintf(stderr,"DLL ERROR: creating laszip writer\n");
        return false;
    }

    // get a pointer to the header of the writer so we can populate it

    laszip_header* header;

    if (laszip_get_header_pointer(laszip_writer, &header))
    {
        fprintf(stderr,"DLL ERROR: getting header pointer from laszip writer\n");
        return false;
    }

    // populate the header

    header->file_source_ID = 4711;
    header->global_encoding = (1<<0);             // see LAS specification for details
    header->version_major = 1;
    header->version_minor = 2;
//    header->file_creation_day = 120;
//    header->file_creation_year = 2013;
    header->point_data_format = 1;
    header->point_data_record_length = 0;
    header->number_of_point_records = pointcloud.size();
    header->number_of_points_by_return[0] = pointcloud.size();
    header->number_of_points_by_return[1] = 0;
    header->point_data_record_length=28;
    header->x_scale_factor = scale;
    header->y_scale_factor = scale;
    header->z_scale_factor = scale;

    header->max_x = max.x();
    header->min_x = min.x();
    header->max_y = max.y();
    header->min_y = min.y();
    header->max_z = max.z();
    header->min_z = min.z();

    // optional: use the bounding box and the scale factor to create a "good" offset


//    fprintf(stderr,"offset_to_point_data after adding two VLRs         : %d\n", (laszip_I32)header->offset_to_point_data);
    // open the writer
    laszip_BOOL compress = (strstr(filename.c_str(), ".laz") != 0);

    if (laszip_open_writer(laszip_writer, filename.c_str(), compress))
    {
        fprintf(stderr,"DLL ERROR: opening laszip writer for '%s'\n", filename.c_str());
        return false;
    }

    fprintf(stderr,"writing file '%s' %scompressed\n", filename.c_str(), (compress ? "" : "un"));

    // get a pointer to the point of the writer that we will populate and write

    laszip_point* point;
    if (laszip_get_point_pointer(laszip_writer, &point))
    {
        fprintf(stderr,"DLL ERROR: getting point pointer from laszip writer\n");
        return false;
    }

    laszip_I64 p_count = 0;
    laszip_F64 coordinates[3];

    for (int i =0; i < pointcloud.size(); i++)
    {
        const auto & p  =pointcloud[i];
        p_count++;
        coordinates[0] = p.x();
        coordinates[1] = p.y();
        coordinates[2] = p.z();
        if (laszip_set_coordinates(laszip_writer, coordinates))
        {
            fprintf(stderr,"DLL ERROR: setting coordinates for point %I64d\n", p_count);
            return false;
        }
        if (i < intensity.size()){
            point->intensity = intensity[i];
        }

        if (laszip_write_point(laszip_writer)) {
            fprintf(stderr, "DLL ERROR: writing point %I64d\n", p_count);
            return false;
        }
    }

    if (laszip_get_point_count(laszip_writer, &p_count))
    {
        fprintf(stderr,"DLL ERROR: getting point count\n");
        return false;
    }

    fprintf(stderr,"successfully written %I64d points\n", p_count);

    // close the writer

    if (laszip_close_writer(laszip_writer))
    {
        fprintf(stderr,"DLL ERROR: closing laszip writer\n");
        return false;
    }

    // destroy the writer

    if (laszip_destroy(laszip_writer))
    {
        fprintf(stderr,"DLL ERROR: destroying laszip writer\n");
        return false;
    }
    return true;
}

bool SaveLAZ::exportPcd(const std::string& filename,
               const std::vector<Eigen::Vector3f>& pointcloud,
               const std::vector<uint16_t>& intensity){
    pcl::PointCloud<pcl::PointXYZI> pointcloud_pcl;
    pointcloud_pcl.resize(pointcloud.size());
    if (pointcloud.size() ==0){
        return false;
    }
    for (int i =0; i < pointcloud.size(); i++)
    {
        pointcloud_pcl[i].getVector3fMap() = pointcloud[i];
        if (i < intensity.size()){
            pointcloud_pcl[i].intensity = intensity[i];
        }
    }
    pcl::io::savePCDFile(filename,pointcloud_pcl,true);
    return true;
}

