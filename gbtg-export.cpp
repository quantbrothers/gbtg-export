// Written by Pavel Fedortsov
// License: do anything what you want

#include <stdexcept>
#include <limits>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <math.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

using namespace boost::filesystem;
using namespace boost::posix_time;
namespace po = boost::program_options;

struct wpt {
    bool new_track;
    double ele;
    double lat;
    double lon;
    double spd;    
};

typedef std::map<ptime, wpt> track_map_t;

const char * c_app_name = "GBTGP-38K(M) gps logger export utility";
const char * c_version_info = "1.0.0";

int wildcmp(const char *wild, const char *string) {
    // Written by Jack Handy - jakkhandy@hotmail.com
    const char *cp = NULL, *mp = NULL;
    while ((*string) && (*wild != '*')) {
        if ((*wild != *string) && (*wild != '?')) {
            return 0;
        }
        wild++; string++;
    }
    while (*string) {
        if (*wild == '*') {
            if (!*++wild) {
                return 1;
            }
            mp = wild;
            cp = string+1;
        } else if ((*wild == *string) || (*wild == '?')) {
            wild++;
            string++;
        } else {
            wild = mp;
            string = cp++;
        }
    }
    while (*wild == '*') {
        wild++;
    }
    return !*wild;
}

void read_track(const std::string & file_name, track_map_t & map_track)
{
    std::ifstream ifs(file_name.c_str(), std::ios::binary);
    if (!ifs.is_open()) {
        throw std::logic_error("Cannot open input file");
    }

    // offset   :   data
    // ------------------------------------
    // 8        :   8-bytes double height
    // 16       :   8-bytes double latitude
    // 24       :   8-bytes double longitude
    // 32       :   80-bytes double speed
    // 41       :   19-bytes string time

    char head[4];
    char block[60];
    ifs.read(head, sizeof(head));
    while (!ifs.eof()) {
        ifs.read(block, sizeof(block));
        wpt p;
        p.new_track = false;
        p.ele = *reinterpret_cast<double*>(block + 8);
        p.lat = *reinterpret_cast<double*>(block + 16);
        p.lon = *reinterpret_cast<double*>(block + 24);
        p.spd = *reinterpret_cast<double*>(block + 32);

        std::string st(block + 41, block + sizeof(block));
        ptime t(time_from_string(st));

        map_track.insert(std::make_pair(t, p));
    }    
    ifs.close();
}


double haversine_km(double lat1, double long1, double lat2, double long2)
{
	const double pi = 3.1415926535897932384626433832795;
	const double d2r = pi / 180.0;
    double dlong = (long2 - long1) * d2r;
    double dlat = (lat2 - lat1) * d2r;
    double a = pow(sin(dlat/2.0), 2) + cos(lat1*d2r) * cos(lat2*d2r) * pow(sin(dlong/2.0), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = 6367 * c;
    return d;
}

void analyze_track(track_map_t & map_track, int split_gap)
{
    if (map_track.empty()) {
        std::cout << "Empty track" << std::endl;		
    } else {
        double total_dist = 0;
        double total_td;
        track_map_t::iterator it_pt = map_track.begin();
        ptime prev_t = map_track.begin()->first;
        wpt prev_p = map_track.begin()->second;
        for ( ; it_pt != map_track.end(); ++it_pt) {
            const ptime & t = it_pt->first;
            wpt & p = it_pt->second;
            time_duration td = (t - prev_t);
            if (0 != split_gap && td > time_duration(seconds(split_gap))) {
                p.new_track = true;
            } else {
                total_td += td.total_seconds();
            }
            total_dist += haversine_km(prev_p.lat, prev_p.lon, p.lat, p.lon);
            prev_t = t;
            prev_p = p;
        }
        std::cout << "First track point time: " << map_track.begin()->first << std::endl;
        std::cout << "Last track point time: " << map_track.rbegin()->first << std::endl;
        std::cout << "Time duration of track: " << total_td << " sec." << std::endl;        
        std::cout << "Total distance: " << total_dist << " Km (" 
            << (total_dist / 1.609344) << " miles)" << std::endl;
    }
}

void write_csv(const std::string & file_name, const track_map_t & map_track)
{
    std::ofstream ofs(file_name.c_str());
    if (!ofs.is_open()) {        
        throw std::logic_error("Cannot open output file");
    }

    track_map_t::const_iterator it_pt = map_track.begin();
    for ( ; it_pt != map_track.end(); ++it_pt) {
        const ptime & t = it_pt->first;
        const wpt & p = it_pt->second;       
        if (it_pt != map_track.begin() && p.new_track) {
            ofs.close();
            path of(file_name);
            ofs.open((of.stem() + "_" + to_iso_string(t) + of.extension()).c_str());
            if (!ofs.is_open()) {
                throw std::logic_error("Cannot open output file");
            }
        }
        ofs << std::setprecision(9)
            << p.lon << ", " << p.lat << ", " << p.ele << ", "
            << p.spd << ", " << to_iso_extended_string(t) << std::endl;
    }
    ofs.close();    
}

void write_gpx(const std::string & file_name,
    const track_map_t & map_track)
{
    std::ofstream ofs(file_name.c_str());
    if (!ofs.is_open()) {
        throw std::logic_error("Cannot open output file");
    }

    ofs << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\" ?>" << std::endl;
    ofs << "<gpx version=\"1.0\" creator=\""
        << c_app_name << " " << c_version_info
        << "\">" << std::endl;

    if (!map_track.empty()) {
        const ptime & min_time = map_track.begin()->first;
        double min_lat = std::numeric_limits<double>::max();
        double max_lat = std::numeric_limits<double>::min();
        double min_lon = std::numeric_limits<double>::max();
        double max_lon = std::numeric_limits<double>::min();
        track_map_t::const_iterator it_pt = map_track.begin();
        for ( ; it_pt != map_track.end(); ++it_pt) {
            const wpt & p = it_pt->second;
            if (p.lat < min_lat) min_lat = p.lat;
            if (p.lat > max_lat) max_lat = p.lat;
            if (p.lon < min_lon) min_lon = p.lon;
            if (p.lon > max_lon) max_lon = p.lon;
        }

        ofs << std::string(4, ' ') << "<time>" << to_iso_extended_string(min_time) << "</time>" << std::endl;
        ofs << std::string(4, ' ') << "<bounds minlat=\"" << min_lat << "\" minlon=\"" << min_lon
            << "\" maxlat=\"" << max_lat << "\" maxlon=\"" << max_lon << "\"/>" << std::endl;

        for (it_pt = map_track.begin(); it_pt != map_track.end(); ++it_pt) {
            const ptime & t = it_pt->first;
            const wpt & p = it_pt->second;
            if (p.new_track) {
                if (it_pt != map_track.begin()) {
                    ofs << std::string(8, ' ') << "</trkseg>" << std::endl;
                    ofs << std::string(4, ' ') << "</trk>" << std::endl;
                }
                ofs << std::string(4, ' ') << "<trk>" << std::endl;
                ofs << std::string(8, ' ') << "<name>" << path(file_name).stem() << " "
                    << to_simple_string(t) << "</name>" << std::endl;
                ofs << std::string(8, ' ') << "<trkseg>" << std::endl;
            }
            ofs << std::string(12, ' ') << "<trkpt lat=\"" << p.lat
                << "\" lon=\"" << p.lon << "\">" << std::endl;                
            ofs << std::string(16, ' ') << "<ele>" << p.ele << "</ele>" << std::endl;
            //ofs << std::string(16, ' ') << "<speed>" << p.spd << "</speed>" << std::endl;
            ofs << std::string(16, ' ') << "<time>" << to_iso_extended_string(t) << "</time>" << std::endl;
            ofs << std::string(12, ' ') << "</trkpt>" << std::endl;
        }
        ofs << std::string(8, ' ') << "</trkseg>" << std::endl;
        ofs << std::string(4, ' ') << "</trk>" << std::endl;
    }
    ofs << "</gpx>" << std::endl;
    ofs.close();
}

void scan_dir(const path & dir, const std::string & pattern,
    bool recursive, track_map_t & map_track)
{
    directory_iterator end_itr; // default construction yields past-the-end
    for (directory_iterator itr(dir); itr != end_itr; ++itr) {
        if (!is_directory(itr->status())) {
            if (0 != wildcmp(pattern.c_str(), itr->path().filename().c_str())) {
                std::cout << "Reading track file: " << itr->path() << std::endl;
                read_track(itr->path().string(), map_track);
            }
        } else if (recursive) {
            scan_dir(itr->path(), pattern, true, map_track);
        }
    }    
}

int usage(const po::options_description & cl_desc,
    char * argv[], const std::string & msg)
{
    std::cout << c_app_name << std::endl
        << "Version: " << c_version_info << std::endl
        << msg << std::endl
        << cl_desc
        << "Example: " << std::endl
        << "  " << path(argv[0]).filename()
        << " -f *.dat -o gpx -F my_trip.gpx" << std::endl;
    return 1;
}

int main(int argc, char * argv[])
{
    po::options_description cl_desc("Command line options");
    cl_desc.add_options()
        ("help,h", "produce help")
        ("input-files,f", po::value< std::vector<std::string> >(), "input file")
        ("output_format,o", po::value<std::string>()->default_value("gpx"),
            "output file format, 'csv' or 'gpx', default is 'gpx'")
        ("output_file,F", po::value<std::string>(), "output file")
        ("recursive,R", "scan for files recursively")
        ("split_track,s", po::value<int>()->default_value(0),
            "maximum allowed time delay in seconds between waypoints, default is infinite. " \
            "Used to split track to subtracks. ");

    try {
        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, cl_desc), vm);
        po::notify(vm);

        if (vm.count("help")) {
            return usage(cl_desc, argv, "");
        }

        if (0 == vm.count("input-files")) {
            return usage(cl_desc, argv, "Input file name was not specified");
        }
        if (0 == vm.count("output_file")) {
            return usage(cl_desc, argv, "Output file name was not specified");
        }

        bool recursive = (0 != vm.count("recursive"));

        track_map_t map_track;
        const std::vector<std::string> & vec_paths =
            vm["input-files"].as< std::vector<std::string> >();
        std::vector<std::string>::const_iterator it_p = vec_paths.begin();
        for ( ; it_p != vec_paths.end(); ++it_p) {
            const std::string & input_path = *it_p;
            std::cout << "Processing input path: " << input_path << std::endl;
            if (exists(input_path)) {
                read_track(input_path, map_track);
            } else {
                path in_path(input_path);
                if (!in_path.has_parent_path()) {
                    in_path = path("./" + in_path.filename());
                }
                scan_dir(in_path.parent_path(), in_path.filename(),
                    recursive, map_track);
            }
        }
        if (!map_track.empty()) {
            map_track.begin()->second.new_track = true;
        }

        std::cout << map_track.size() << " waypoints." << std::endl;

        int split_gap = vm["split_track"].as<int>();        
        analyze_track(map_track, split_gap);        

        const std::string & fmt = vm["output_format"].as<std::string>();
        const std::string & output_path = vm["output_file"].as<std::string>();
        if (fmt == "csv") {
            std::cout << "Converting waypoints to CSV..." << std::endl;
            write_csv(output_path, map_track);
        } else {
            std::cout << "Converting waypoints to GPX track..." << std::endl;
            write_gpx(output_path, map_track);
        }
        std::cout << "Done" << std::endl;
    } catch (po::error & exc) {
        std::cout << "Command line error: " << exc.what() << std::endl;
        return -1;
    } catch (filesystem_error & exc) {
        std::cout << "Filesystem error: " << exc.what() << std::endl;
        return -1;
    } catch (std::exception & exc) {
        std::cout << "Error: " << exc.what() << std::endl;
        return -1;
    }
    return 0;
}
