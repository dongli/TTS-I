load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_code.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_csm.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/contributed.ncl"

load "/Users/dongli/Works/TTS/Tools/tts_plot_utils.ncl"

begin

    ; --------------------------------------------------------------------------
    ; user configurations
    is_plot_polygons = True
    is_plot_quantities = True
    is_plot_vertices = True
    is_plot_vertex_number = False
    is_plot_polygon_area = False
    is_plot_testpoint = False
    is_plot_point_counters = False
    projection = "CE"
    projection@angle = 10.0
    line_width = 0.1
    point_size = 0.001

    ; --------------------------------------------------------------------------
    file_name_pattern = ask("Input the polygon file name pattern:")
    polygon_file_names = systemfunc("ls "+file_name_pattern)
    counter_file_names = systemfunc("ls debug_counters*.nc 2> /dev/null")

    ; --------------------------------------------------------------------------
    res_map = True
    setup_map(projection, res_map)

    ; --------------------------------------------------------------------------
    if (projection .eq. "ST") then
        res_map@mpCenterLonF = 0.0
        res_map@mpCenterLatF = 0.0
    end if

    ; --------------------------------------------------------------------------
    if (is_plot_polygons .or. is_plot_quantities .or. \
        is_plot_vertices .or. is_plot_point_counters) then
        do k = 0, dimsizes(polygon_file_names)-1
            wks = gsn_open_wks("newpdf", systemfunc("basename -s .nc "+ \
                polygon_file_names(k))+"_"+projection)

            system("echo '-----------------------------------------'")
            if (is_plot_point_counters) then
                system("echo plotting "+polygon_file_names(k)+" and "+ \
                    counter_file_names(k))
            else
                system("echo plotting "+polygon_file_names(k))
            end if

            if (projection .eq. "ST") then
                if (isatt(projection, "move_viewpoint") .and. \
                    projection@move_viewpoint) then
                    get_viewpoint(res_map@mpCenterLonF, res_map@mpCenterLatF)
                end if
            end if
                
            map = gsn_csm_map(wks, res_map)

            f = addfile(polygon_file_names(k), "r")

            if (is_plot_point_counters) then
                plot = show_point_counters(wks, counter_file_names(k))
                overlay(map, plot)
            end if

            draw(map)

            if (is_plot_polygons) then
                show_polygons(wks, map, \
                    f->new_vertex_lon, f->new_vertex_lat, \
                    f->first_point_idx, f->second_point_idx, \
                    f->edge_num, f->edge_idx, f->edge_ont, \
                    f->new_testpoint_lon, f->new_testpoint_lat, \
                    point_size, line_width, is_plot_testpoint)
            end if

            if (is_plot_quantities) then
                ;gsn_define_colormap(wks, "WhBlGrYeRe")
                gsn_define_colormap(wks, "WhBlReWh")
                cmap = gsn_retrieve_colormap(wks)
                ;num_level = 13
                num_level = 21
                stride = (dimsizes(cmap(:,0))-3.0)/num_level
                levels = new(num_level, float)
                ;levels = fspan(-0.05, 1.15, num_level)
                levels = fspan(0.5, 1.5, num_level)
                colors = new((/num_level+3,3/), float)
                colorlabels = new(num_level, string)
                colors(0:1,:) = cmap(0:1,:)
                do i = 0, num_level
                    colors(i+2,:) = cmap(floattoint(2+i*stride),:)
                end do
                do i = 0, num_level-1
                    colorlabels(i) = sprintf("%5.2f", levels(i))
                end do
                res_lb = True
                res_lb@lbOrientation = "Horizontal"
                res_lb@vpHeightF = 0.1
                res_lb@vpWidthF = 0.7
                res_lb@lbLabelAngleF = 45.0
                res_lb@lbLabelFontHeightF = 0.015
                res_lb@lbAutoManage = False
                res_lb@lbFillColors = colors(2:num_level+2,:)
                res_lb@lbPerimOn = False
                res_lb@lbMonoFillPattern = True
                gsn_labelbar_ndc(wks, num_level, colorlabels, 0.13, 0.25, res_lb)

                show_polygon_quantities(wks, map, levels, colors, \
                    f->new_vertex_lon, f->new_vertex_lat, \
                    f->first_point_idx, f->second_point_idx, \
                    f->edge_num, f->edge_idx, f->edge_ont, f->q0)
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
        wks = gsn_open_wks("pdf", "vertex_number")
        show_vertex_number(wks, polygon_file_names)
    end if

    if (is_plot_polygon_area) then
        show_polygon_area(wks, file_names, 0)
    end if

end
