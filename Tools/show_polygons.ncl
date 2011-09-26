load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_code.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_csm.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/contributed.ncl"

load "/Users/dongli/Works/TTS/Tools/tts_plot_utils.ncl"

begin

    is_plot_polygons = True
    is_plot_vertices = False
    is_plot_vertex_number = False
    is_plot_polygon_area = False
    is_plot_testpoint = False
    is_plot_point_counters = False
    projection = "CE"
    projection@min_lat = 80.0
    projection@max_lat = -80.0

    line_width = 1.0
    point_size = 0.0005

    polygon_file_names = systemfunc("ls tts_test00120.nc")
    counter_file_names = systemfunc("ls debug_counters*.nc 2> /dev/null")

    res_map = setup_map(projection)

    if (is_plot_polygons .or. is_plot_vertices .or. is_plot_point_counters) then
        do i = 0, dimsizes(polygon_file_names)-1
            wks = gsn_open_wks("newpdf", systemfunc("basename -s .nc "+polygon_file_names(i))+"_"+projection)

            system("echo '-----------------------------------------'")
            if (is_plot_point_counters) then
                system("echo plotting "+polygon_file_names(i)+" and "+ \
                    counter_file_names(i))
            else
                system("echo plotting "+polygon_file_names(i))
            end if

            map = gsn_csm_map(wks, res_map)

            f = addfile(polygon_file_names(i), "r")

            if (is_plot_point_counters) then
                plot = show_point_counters(wks, counter_file_names(i))
                overlay(map, plot)
            end if

            draw(map)

            if (is_plot_polygons) then
                show_polyons(wks, map, \
                    f->new_vertex_lon, f->new_vertex_lat, \
                    f->first_point_idx, f->second_point_idx, \
                    f->edge_num, f->edge_idx, f->edge_ont, \
                    f->new_testpoint_lon, f->new_testpoint_lat, \
                    point_size, line_width, is_plot_testpoint)
            end if

            if (is_plot_vertices) then
                show_vertices(wks, map, \
                    f->new_vertex_lon, f->new_vertex_lat, \
                    f->new_testpoint_lon, f->new_testpoint_lat, \
                    point_size, is_plot_testpoint)
            end if

            frame(wks)

            delete(f)
            delete(wks)
        end do
    end if

    if (is_plot_vertex_number) then
        show_vertex_number(wks, file_names)
    end if

    if (is_plot_polygon_area) then
        show_polygon_area(wks, file_names, 0)
    end if

end
