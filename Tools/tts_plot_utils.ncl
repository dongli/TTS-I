function calc_polygon_center_lon(lon)
begin

    if (lon(0) .lt. 360.0-lon(0)) then
        sign = -1.0
    else
        sign = 1.0
    end if
    center_lon = lon(0)
    prev_lon = lon(0)
    do i = 1, dimsizes(lon)-1
        center_lon = center_lon+lon(i)
        prev_lon = lon(i)
        dist1 = abs(lon(i)-prev_lon)
        dist2 = 360.0-dist1
        if (dist1 .gt. dist2) then
            center_lon = center_lon+sign*360.0
        end if
        if (sign .eq. -1.0) then
            if (lon(i) .gt. 360.0-lon(i)) then
                prev_lon = prev_lon-360.0
            end if
        else
            if (lon(i) .lt. 360.0-lon(i)) then
                prev_lon = prev_lon+360.0
            end if
        end if
    end do
    center_lon = center_lon/dimsizes(lon)

    if (center_lon .lt. 0.0) then
        center_lon = center_lon+360.0
    end if
    if (center_lon .gt. 360.0) then
        center_lon = center_lon-360.0
    end if

    return center_lon

end

function setup_map(projection)
local res
begin

    res                         = True
    res@gsnDraw                 = False
    res@gsnFrame                = False
    res@gsnMaximize             = True
    res@mpGreatCircleLinesOn    = True
    res@mpFillOn                = False
    ;res@mpCenterLonF            = 180.0
    if (projection .eq. "NH") then
        if (.not. isatt(projection, "min_lat")) then
            system("echo [Error]: setup_map: projection@min_lat should be defined for NH projection.")
            exit
        end if
        res@mpMinLatF           = projection@min_lat
        res@gsnPolar            = "NH"
    end if
    if (projection .eq. "SH") then
        if (.not. isatt(projection, "max_lat")) then
            system("echo [Error]: setup_map: projection@max_lat should be defined for SH projection.")
            exit
        end if
        res@mpMaxLatF           = projection@max_lat
        res@gsnPolar            = "SH"
    end if
    if (projection .eq. "ST") then
        if (.not. isatt(projection, "angle")) then
            system("echo [Error]: setup_map: projection@angle should be defined for ST projection.")
            exit
        end if
        res@mpProjection        = "satellite"
        res@mpLimitMode         = "angles"
        res@mpLeftAngleF        = projection@angle
        res@mpRightAngleF       = projection@angle
        res@mpBottomAngleF      = projection@angle
        res@mpTopAngleF         = projection@angle
    end if

    return res

end

function show_velocity_contour(wks, u_or_v)
local res, plot
begin

    gsn_define_colormap(wks, "WhiteBlueGreenYellowRed")
    res                         = True
    res@gsnDraw                 = False
    res@gsnFrame                = False
    ;res@cnFillOn                = True
    ;res@cnLinesOn               = False
    res@cnLineLabelsOn           = False
    ;res@gsnSpreadColors         = True

    plot = gsn_csm_contour(wks, u_or_v, res)

    return plot

end

function show_velocity_speed(wks, u, v)
local res, plot
begin

    gsn_define_colormap(wks, "WhiteBlueGreenYellowRed")
    res                         = True
    res@gsnMaximize             = True
    res@gsnDraw                 = False
    res@gsnFrame                = False
    res@cnFillOn                = True
    res@cnLinesOn               = False
    res@gsnSpreadColors         = True

    speed = sqrt(u*u+v*v)
    copy_VarCoords(u, speed)

    plot = gsn_csm_contour(wks, speed, res)

    return plot

end

function show_velocity_vector(wks, u, v)
local res, plot
begin

    res                         = True
    res@gsnDraw                 = False
    res@gsnFrame                = False
    res@vcRefAnnoOrthogonalPosF = -1.0
    res@vcRefMagnitudeF         = 0.5
    res@vcRefLengthF            = 0.045
    res@vcGlyphStyle            = "CurlyVector"
    res@vcMinDistanceF          = 0.017
    res@vcVectorDrawOrder       = "PostDraw"
    res@vcRefAnnoOn             = False

    plot = gsn_csm_vector(wks, u, v, res)

    return plot

end

procedure show_vertices(wks, map, vertex_lon, vertex_lat, \
    testpoint_lon, testpoint_lat, point_size, is_plot_testpoint)
local res, num_total_vertex, num_total_testpoint
begin

    res                 = True
    res@gsMarkerIndex   = 1
    res@gsMarkerSizeF   = point_size

    system("echo Summary:")
    num_total_vertex = dimsizes(vertex_lon)
    system("echo   num_total_vertex     = "+num_total_vertex)

    res@gsMarkerColor   = "black"
    gsn_polymarker(wks, map, vertex_lon, vertex_lat, res)

    if (is_plot_testpoint) then
        num_total_testpoint = dimsizes(testpoint_lon)
        system("echo   num_total_testpoint  = "+num_total_testpoint)

        res@gsMarkerColor   = "red"
        gsn_polymarker(wks, map, testpoint_lon, testpoint_lat, res)
    end if

end

procedure show_polygon(wks, map, vertex_lon, vertex_lat, \
    testpoint_lon, testpoint_lat, line_width, is_plot_testpoint)
local res, num_edge, allpoint_lon, allpoint_lat
begin

    res                     = True
    res@gsLineThicknessF    = line_width
    res@gsLineColor         = "blue"

    num_edge = dimsizes(testpoint_lon)

    res@gsLineDashPattern = 0
    gsn_polyline(wks, map, vertex_lon, vertex_lat, res)

    if (is_plot_testpoint) then
        allpoint_lon = new(2*num_edge+1, double)
        allpoint_lat = new(2*num_edge+1, double)

        allpoint_lon(0:2*num_edge-2:2) = (/ vertex_lon(0:num_edge-1) /)
        allpoint_lat(0:2*num_edge-2:2) = (/ vertex_lat(0:num_edge-1) /)
        allpoint_lon(1:2*num_edge-1:2) = (/ testpoint_lon /)
        allpoint_lat(1:2*num_edge-1:2) = (/ testpoint_lat /)
        allpoint_lon(2*num_edge) = allpoint_lon(0)
        allpoint_lat(2*num_edge) = allpoint_lat(0)

        res@gsLineDashPattern = 2
        gsn_polyline(wks, map, allpoint_lon, allpoint_lat, res)

        delete(allpoint_lon)
        delete(allpoint_lat)
    end if

end

procedure show_polyons(wks, map, vertex_lon, vertex_lat, \
    first_point_idx, second_point_idx, edge_num, edge_idx, edge_ont, \
    testpoint_lon, testpoint_lat, point_size, line_width, is_plot_testpoint)
local res, lon, lat, i, j, k, l
begin

    num_total_vertex = dimsizes(vertex_lon)
    num_total_polygon = dimsizes(edge_num)
    num_total_edge = dimsizes(first_point_idx)

    system("echo Summary:")
    system("echo   num_total_vertex  = "+num_total_vertex)
    system("echo   num_total_polygon = "+num_total_polygon)
    system("echo   num_total_edge    = "+num_total_edge)

    l = 0
    do i = 0, num_total_polygon-1
        vtxlon = new(edge_num(i)+1, double)
        vtxlat = new(edge_num(i)+1, double)
        testlon = new(edge_num(i), double)
        testlat = new(edge_num(i), double)
        do j = 0, edge_num(i)-1
            if (edge_ont(l) .eq. 0) then ; left
                k = first_point_idx(edge_idx(l)-1)-1
            else ; right
                k = second_point_idx(edge_idx(l)-1)-1
            end if
            vtxlon(j) = vertex_lon(k)
            vtxlat(j) = vertex_lat(k)
            testlon(j) = testpoint_lon(edge_idx(l)-1)
            testlat(j) = testpoint_lat(edge_idx(l)-1)
            l = l+1
        end do
        vtxlon(edge_num(i)) = vtxlon(0)
        vtxlat(edge_num(i)) = vtxlat(0)

        show_polygon(wks, map, vtxlon, vtxlat, testlon, testlat, \
            line_width, is_plot_testpoint)

        delete(vtxlon)
        delete(vtxlat)
        delete(testlon)
        delete(testlat)
    end do

end

procedure show_vertex_number(wks, file_names)
local num_time, num_total_vertex, i, f, res, plot
begin

    num_time = dimsizes(file_names)
    num_total_vertex = new(num_time, integer)

    do i = 0, num_time-1
        f = addfile(file_names(i), "r")
        num_total_vertex(i) = dimsizes(f->vertex_lon)
        delete(f)
    end do

    res                     = True
    res@gsnMaximize         = True
    res@tiMainString        = "Total Number of Polygon Vertices"
    res@tiXAxisString       = "time step"
    res@trYMaxF             = 2000

    plot = gsn_csm_xy(wks, ispan(0, num_time-1, 1), num_total_vertex, res)

end

procedure show_polygon_area(wks, file_names, idx)
local num_time, area, i, f, res, plot
begin

    num_time = dimsizes(file_names)
    area = new(num_time, double)

    do i = 0, num_time-1
        f = addfile(file_names(i), "r")
        area(i) = f->area(idx)
        delete(f)
    end do

    res                     = True
    res@gsnMaximize         = True
    res@tiMainString        = "Area of Polygon "+idx
    res@tiXAxisString       = "time step"
    res@trYMaxF             = 0.031
    res@trYMinF             = 0.030

    plot = gsn_csm_xy(wks, ispan(0, num_time-1, 1), area, res)

end

function show_point_counters(wks, file_name)
local res, f, lon_bnds, lat_bnds, plot
begin

    f = addfile(file_name, "r")

    gsn_define_colormap(wks, "WhiteBlueGreenYellowRed")

    res                         = True
    res@gsnDraw                 = False
    res@gsnFrame                = False
    res@gsnSpreadColors         = True
    res@gsnLeftString           = ""
    ;res@lbLabelBarOn            = False
    res@cnFillOn                = True
    res@cnFillMode              = "CellFill"
    res@cnCellFillEdgeColor     = "grey"
    res@cnLinesOn               = False
    res@cnLineLabelsOn          = False
    ;res@cnLevelSelectionMode    = "ManualLevels"
    ;res@cnMinLevelValF          = 0
    ;res@cnMaxLevelValF          = 50
    ;res@cnMaxLevelValF          = 1
    dims = dimsizes(f->lon_bnds)
    lon_bnds = new(dims(0)+1, double)
    do i = 0, dims(0)-1
        lon_bnds(i) = f->lon_bnds(i,0)
    end do
    lon_bnds(dimsizes(lon_bnds)-1) = f->lon_bnds(dims(0)-1,1)
    res@sfXArray = lon_bnds
    dims = dimsizes(f->lat_bnds)
    lat_bnds = new(dims(0)+1, double)
    do j = 0, dims(0)-1
        lat_bnds(j) = f->lat_bnds(j,0)
    end do
    lat_bnds(dimsizes(lat_bnds)-1) = f->lat_bnds(dims(0)-1,1)
    res@sfYArray = lat_bnds

    plot = gsn_csm_contour(wks, f->counters, res)

    delete(f)
    delete(lon_bnds)
    delete(lat_bnds)

    return plot

end
