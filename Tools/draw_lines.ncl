load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_code.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_csm.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/contributed.ncl"

load "/Users/dongli/Works/TTS/Tools/tts_plot_utils.ncl"

begin

    projection = "ST"
    projection@angle = 1.0

    if (.not. isdefined("file_name")) then
        system("echo Input the node coordinates file:")
        file_name = systemfunc("read -e -p ' > ' ans; echo $ans")
    end if

    if (projection .eq. "ST") then
        if (.not. isatt(projection, "angle")) then
            system("echo Input view angle:")
            projection@angle = stringtofloat(systemfunc \
                ("read -p ' > ' ans; echo $ans"))
        end if
    end if

    contents = asciiread(file_name, -1, "double")

    num_point = floattoint(dimsizes(contents)/2.0)
    point_lon = new(num_point, double)
    point_lat = new(num_point, double)

    point_lon = contents(0:dimsizes(contents)-1:2)
    point_lat = contents(1:dimsizes(contents)-1:2)

    Rad2Deg = 45.0/atan(1.0)
    point_lon = point_lon*Rad2Deg
    point_lat = point_lat*Rad2Deg

    ; --------------------------------------------------------------------------
    wks = gsn_open_wks("pdf", file_name)

    projection@center_lon = new(1, double)
    projection@center_lat = new(1, double)
    calc_polygon_center(point_lon, point_lat, \
                        projection@center_lon, projection@center_lat)
    res_map = True
    setup_map(projection, res_map)
    map = gsn_csm_map(wks, res_map)

    draw(map)

    ; --------------------------------------------------------------------------
    res_line                        = True
    res_line@gsLineThicknessF       = 0.5
    res_line@gsLineColor            = "blue"

    res_point                       = True
    res_point@gsMarkerIndex         = 1
    res_point@gsMarkerSizeF         = 0.01

    gsn_polyline(wks, map, point_lon, point_lat, res_line)

    gsn_polymarker(wks, map, point_lon, point_lat, res_point)

    frame(wks)

end
