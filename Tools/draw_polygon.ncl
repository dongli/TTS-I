load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_code.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_csm.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/contributed.ncl"

load "/Users/dongli/Works/TTS/Tools/tts_plot_utils.ncl"

begin

    projection = "ST"
    projection@angle = 0.5
    line_width = 1.0
    point_size = 0.01

    if (.not. isdefined("file_name")) then
        system("echo Input the polygon file:")
        file_name = systemfunc("read -e -p ' > ' ans; echo $ans")
    end if

    Rad2Deg = 45.0/atan(1.0)

    content = asciiread(file_name, -1, "double")
    num_vertex = dimsizes(content)/2
    vertex_lon = new(num_vertex+1, double)
    vertex_lat = new(num_vertex+1, double)
    vertex_lon(0:num_vertex-1) = content(0:dimsizes(content)-1:2)*Rad2Deg
    vertex_lat(0:num_vertex-1) = content(1:dimsizes(content)-1:2)*Rad2Deg
    vertex_lon(num_vertex) = vertex_lon(0)
    vertex_lat(num_vertex) = vertex_lat(0)

    res_map = setup_map(projection)
    res_map@mpCenterLonF = calc_polygon_center_lon(vertex_lon)
    res_map@mpCenterLatF = avg(vertex_lat)

    res_line                        = True
    res_line@gsLineThicknessF       = line_width
    res_line@gsLineColor            = "blue"

    res_point                       = True
    res_point@gsMarkerIndex         = 1
    res_point@gsMarkerSizeF         = point_size

    wks = gsn_open_wks("pdf", file_name)

    map = gsn_csm_map(wks, res_map)

    gsn_polyline(wks, map, vertex_lon, vertex_lat, res_line)

    gsn_polymarker(wks, map, vertex_lon, vertex_lat, res_point)

    frame(wks)

end
