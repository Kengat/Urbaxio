#include "cad_kernel/MeshBuffers.h"
#include "cad_kernel/cad_kernel.h"

// ---                       OpenCascade ---
#include <Standard_Macro.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopLoc_Location.hxx>
#include <TopAbs.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <gp_XYZ.hxx>
#include <TColgp_Array1OfPnt.hxx>      //                  
#include <TShort_Array1OfShortReal.hxx> //              (           )
#include <TColgp_HArray1OfPnt.hxx>      // Handle                  
#include <TShort_HArray1OfShortReal.hxx>// Handle                     
#include <TColgp_HArray1OfPnt2d.hxx>
#include <TColgp_Array1OfPnt2d.hxx>
#include <Poly_Array1OfTriangle.hxx>   //                          
#include <Message.hxx>
#include <BRepLProp_SLProps.hxx>
#include <Precision.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Vertex.hxx>

#include <cmath>
#include <vector>
#include <iostream>

namespace Urbaxio::CadKernel {

    MeshBuffers TriangulateShape(const TopoDS_Shape& shape,
        double linDefl,
        double angDefl)
    {
        MeshBuffers out;
        if (shape.IsNull()) { /*...*/ return out; }

        // --- 1.                             ---
        if (linDefl <= 0.0) { /* ... (            linDefl        ) ... */ }
        //             linDefl:
        if (linDefl <= 0.0)
        {
            Bnd_Box bb;
            try { BRepBndLib::Add(shape, bb, false); }
            catch (...) {
                try { BRepBndLib::Add(shape, bb, true); }
                catch (...) {}
            }
            if (bb.IsVoid()) { linDefl = 0.1; }
            else {
                double diag = bb.SquareExtent() > 1.e-16 ? std::sqrt(bb.SquareExtent()) : 1.0;
                linDefl = diag * 0.002;
                if (linDefl < 1e-6) linDefl = 1e-6;
            }
            Message::SendInfo() << "TriangulateShape: Using linear deflection: " << linDefl << ", angular deflection: " << angDefl;
        }

        // --- 2.                     ---
        Standard_ErrorHandler aErrHandler;
        try {
            OCC_CATCH_SIGNALS
                BRepTools::Clean(shape);
            BRepMesh_IncrementalMesh mesher(shape, linDefl, Standard_False, angDefl, Standard_True);
            if (!mesher.IsDone()) { /* ... */ }
            else { /* ... */ }
        }
        catch (Standard_Failure& e) { /* ... */ return out; }
        catch (...) { /* ... */ return out; }

        // --- 3.                                       ---
        TopExp_Explorer faceExplorer(shape, TopAbs_FACE);
        int faceIndex = 0;
        for (; faceExplorer.More(); faceExplorer.Next())
        {
            ++faceIndex;
            const TopoDS_Face& face = TopoDS::Face(faceExplorer.Current());
            TopLoc_Location location;
            Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, location);

            if (triangulation.IsNull() || triangulation->NbNodes() == 0 || triangulation->NbTriangles() == 0) {
                continue; //                               
            }

            const gp_Trsf& transformation = location.Transformation();
            // Diagnostics: check transformation handedness (mirror)
            gp_Vec vx(1,0,0), vy(0,1,0), vz(0,0,1);
            vx.Transform(transformation); vy.Transform(transformation); vz.Transform(transformation);
            const double detSign = (vx.Crossed(vy)).Dot(vz);
            // --- Topological fallback for planar quads that collapsed to a triangle ---
            BRepAdaptor_Surface saFace(face, Standard_True);
            const bool isPlanar = (saFace.GetType() == GeomAbs_Plane);
            if (isPlanar && (!triangulation.IsNull() && triangulation->NbNodes() < 4)) {
                std::vector<gp_Pnt> wirePts;
                try {
                    TopoDS_Wire outer = BRepTools::OuterWire(face);
                    if (!outer.IsNull()) {
                        BRepTools_WireExplorer wx(outer);
                        TopoDS_Vertex lastV;
                        for (; wx.More(); wx.Next()) {
                            TopoDS_Vertex v = wx.CurrentVertex();
                            if (!lastV.IsNull() && v.IsSame(lastV)) continue;
                            gp_Pnt p = BRep_Tool::Pnt(v);
                            p.Transform(location.Transformation());
                            if (!wirePts.empty() && p.Distance(wirePts.back()) < Precision::Confusion()) continue;
                            wirePts.push_back(p);
                            lastV = v;
                        }
                        if (wirePts.size() >= 2 && wirePts.front().Distance(wirePts.back()) < Precision::Confusion()) wirePts.pop_back();
                    }
                } catch (...) {}

                if (wirePts.size() >= 4) {
                    const unsigned int base = static_cast<unsigned int>(out.vertices.size() / 3);
                    auto pushV = [&](const gp_Pnt& p){
                        out.vertices.push_back(static_cast<float>(p.X()));
                        out.vertices.push_back(static_cast<float>(p.Y()));
                        out.vertices.push_back(static_cast<float>(p.Z()));
                    };
                    const gp_Pnt& q0 = wirePts[0];
                    const gp_Pnt& q1 = wirePts[1];
                    const gp_Pnt& q2 = wirePts[2];
                    const gp_Pnt& q3 = wirePts[3];
                    pushV(q0); pushV(q1); pushV(q2); pushV(q3);
                    auto area2sq = [](const gp_Pnt& A, const gp_Pnt& B, const gp_Pnt& C){ gp_Vec a(A,B), b(A,C); return a.Crossed(b).SquareMagnitude(); };
                    const double minA = std::min(area2sq(q0,q1,q2), area2sq(q0,q2,q3));
                    const double minB = std::min(area2sq(q0,q1,q3), area2sq(q1,q2,q3));
                    auto emit = [&](unsigned a, unsigned b, unsigned c){ unsigned i1=base+a, i2=base+b, i3=base+c; if (face.Orientation()==TopAbs_REVERSED) std::swap(i2,i3); out.indices.push_back(i1); out.indices.push_back(i2); out.indices.push_back(i3); };
                    if (minB > minA) { emit(0,1,3); emit(1,2,3); } else { emit(0,1,2); emit(0,2,3); }
                    Message::SendInfo() << "[TRI][face " << faceIndex << "] topo-fallback quad from wire (nodes<4)";
                    continue;
                }
            }

            // --- Node/triangle access ---
            const Handle(TColgp_HArray1OfPnt) hNodes = triangulation->MapNodeArray();
            const TColgp_Array1OfPnt& nodes = hNodes->Array1();
            const Poly_Array1OfTriangle& triangles = triangulation->Triangles();
            // UV nodes for surface-normal based winding test
            const bool hasUV = triangulation->HasUVNodes();
            Handle(TColgp_HArray1OfPnt2d) hUVNodes;
            if (hasUV) { hUVNodes = triangulation->MapUVNodeArray(); }
            const TColgp_Array1OfPnt2d* uvArr = (!hUVNodes.IsNull()) ? &hUVNodes->Array1() : nullptr;
            BRepAdaptor_Surface surfAdaptor(face, Standard_True);

            const unsigned int baseVertexIndex = static_cast<unsigned int>(out.vertices.size() / 3);

            // --- 5. Push transformed vertices (normals will be recomputed globally) ---
            for (Standard_Integer i = nodes.Lower(); i <= nodes.Upper(); ++i) {
                gp_Pnt vertex = nodes(i);
                vertex.Transform(transformation);
                out.vertices.push_back(static_cast<float>(vertex.X()));
                out.vertices.push_back(static_cast<float>(vertex.Y()));
                out.vertices.push_back(static_cast<float>(vertex.Z()));
            }

            // --- 6. Indices, consistent with face orientation ---
            const bool reversed = (face.Orientation() == TopAbs_REVERSED);
            int triEmitted = 0, triOOB = 0, triDegenerate = 0;
            int triSwap = 0, triNoProps = 0, triZeroSurfN = 0;
            for (Standard_Integer i = triangles.Lower(); i <= triangles.Upper(); ++i) {
                Standard_Integer n1, n2, n3;
                triangles(i).Get(n1, n2, n3);
                unsigned int idx1 = baseVertexIndex + n1 - 1;
                unsigned int idx2 = baseVertexIndex + n2 - 1;
                unsigned int idx3 = baseVertexIndex + n3 - 1;
                unsigned int max_index = baseVertexIndex + nodes.Length() - 1;
                if (idx1 > max_index || idx2 > max_index || idx3 > max_index) { triOOB++; continue; }
                // Degeneracy check in world space
                gp_Pnt p1 = nodes(n1); p1.Transform(transformation);
                gp_Pnt p2 = nodes(n2); p2.Transform(transformation);
                gp_Pnt p3 = nodes(n3); p3.Transform(transformation);
                gp_Vec e1(p1, p2), e2(p1, p3);
                if (e1.Crossed(e2).SquareMagnitude() <= 1e-24) { triDegenerate++; }
                // Decide winding using surface normal at UV center when available
                bool doSwap = reversed; // default legacy
                if (uvArr != nullptr) {
                    const gp_Pnt2d& uv1 = (*uvArr)(n1);
                    const gp_Pnt2d& uv2 = (*uvArr)(n2);
                    const gp_Pnt2d& uv3 = (*uvArr)(n3);
                    gp_Pnt2d uvc( (uv1.X()+uv2.X()+uv3.X())/3.0, (uv1.Y()+uv2.Y()+uv3.Y())/3.0 );
                    BRepLProp_SLProps props(surfAdaptor, uvc.X(), uvc.Y(), 1, 1e-6);
                    if (props.IsNormalDefined()) {
                        gp_Dir nSurf = props.Normal();
                        if (reversed) nSurf.Reverse();
                        // Transform surface normal to world
                        gp_Vec nSurfV(nSurf.X(), nSurf.Y(), nSurf.Z());
                        nSurfV.Transform(transformation);
                        if (nSurfV.SquareMagnitude() > Precision::SquareConfusion()) {
                            gp_Dir nSurfW(nSurfV);
                            gp_Dir nTriW(e1.Crossed(e2));
                            doSwap = (nTriW.Dot(nSurfW) < 0.0);
                        }
                        else { triZeroSurfN++; }
                    }
                    else { triNoProps++; }
                } else if (surfAdaptor.GetType() == GeomAbs_Plane) {
                    gp_Dir nSurf = gp_Dir(surfAdaptor.Plane().Axis().Direction());
                    if (reversed) nSurf.Reverse();
                    gp_Vec nSurfV(nSurf.X(), nSurf.Y(), nSurf.Z());
                    nSurfV.Transform(transformation);
                    if (nSurfV.SquareMagnitude() > Precision::SquareConfusion()) {
                        gp_Dir nSurfW(nSurfV);
                        gp_Dir nTriW(e1.Crossed(e2));
                        doSwap = (nTriW.Dot(nSurfW) < 0.0);
                    }
                    else { triZeroSurfN++; }
                }
                if (doSwap) { std::swap(idx2, idx3); triSwap++; }
                out.indices.push_back(idx1);
                out.indices.push_back(idx2);
                out.indices.push_back(idx3);
                triEmitted++;
            }
            // Per-face summary
            BRepAdaptor_Surface sa(face, Standard_True);
            const char* surfType = "Unknown";
            switch (sa.GetType()) {
                case GeomAbs_Plane: surfType = "Plane"; break;
                case GeomAbs_Cylinder: surfType = "Cylinder"; break;
                case GeomAbs_Cone: surfType = "Cone"; break;
                case GeomAbs_Sphere: surfType = "Sphere"; break;
                case GeomAbs_Torus: surfType = "Torus"; break;
                case GeomAbs_BezierSurface: surfType = "Bezier"; break;
                case GeomAbs_BSplineSurface: surfType = "BSpline"; break;
                default: break;
            }
            const int triCount = triangles.Upper() - triangles.Lower() + 1;
            Message::SendInfo() << "[TRI][face " << faceIndex << "] nodes=" << nodes.Length()
                << " tris=" << triCount
                << " emitted=" << triEmitted
                << " oob=" << triOOB
                << " deg~0=" << triDegenerate
                << " swaps=" << triSwap
                << " noProps=" << triNoProps
                << " zeroSurfN=" << triZeroSurfN
                << " hasUV=" << (triangulation->HasUVNodes() ? "Y" : "N")
                << " reversed=" << (reversed ? "Y" : "N")
                << " mirroredXform=" << ((detSign < 0.0) ? "Y" : "N")
                << " surf=" << surfType;
        }

        // --- 7. Recompute vertex normals from indexed triangles (world space) ---
        if (!out.vertices.empty() && !out.indices.empty()) {
            out.normals.assign(out.vertices.size(), 0.0f);
            const size_t vcount = out.vertices.size() / 3;
            for (size_t i = 0; i + 2 < out.indices.size(); i += 3) {
                const unsigned int i0 = out.indices[i];
                const unsigned int i1 = out.indices[i + 1];
                const unsigned int i2 = out.indices[i + 2];
                if (i0 >= vcount || i1 >= vcount || i2 >= vcount) continue;
                gp_XYZ p0(out.vertices[i0*3+0], out.vertices[i0*3+1], out.vertices[i0*3+2]);
                gp_XYZ p1(out.vertices[i1*3+0], out.vertices[i1*3+1], out.vertices[i1*3+2]);
                gp_XYZ p2(out.vertices[i2*3+0], out.vertices[i2*3+1], out.vertices[i2*3+2]);
                gp_XYZ v1 = p1 - p0;
                gp_XYZ v2 = p2 - p0;
                gp_XYZ fn = v1 ^ v2; // cross
                if (fn.SquareModulus() <= 1e-24) continue;
                out.normals[i0*3+0] += static_cast<float>(fn.X());
                out.normals[i0*3+1] += static_cast<float>(fn.Y());
                out.normals[i0*3+2] += static_cast<float>(fn.Z());
                out.normals[i1*3+0] += static_cast<float>(fn.X());
                out.normals[i1*3+1] += static_cast<float>(fn.Y());
                out.normals[i1*3+2] += static_cast<float>(fn.Z());
                out.normals[i2*3+0] += static_cast<float>(fn.X());
                out.normals[i2*3+1] += static_cast<float>(fn.Y());
                out.normals[i2*3+2] += static_cast<float>(fn.Z());
            }
            for (size_t vi = 0; vi < vcount; ++vi) {
                gp_XYZ n(out.normals[vi*3+0], out.normals[vi*3+1], out.normals[vi*3+2]);
                const double m2 = n.SquareModulus();
                if (m2 > 1e-24) {
                    const double inv = 1.0 / std::sqrt(m2);
                    out.normals[vi*3+0] = static_cast<float>(n.X() * inv);
                    out.normals[vi*3+1] = static_cast<float>(n.Y() * inv);
                    out.normals[vi*3+2] = static_cast<float>(n.Z() * inv);
                } else {
                    out.normals[vi*3+0] = 0.0f;
                    out.normals[vi*3+1] = 0.0f;
                    out.normals[vi*3+2] = 1.0f;
                }
            }
        }

        Message::SendInfo() << "TriangulateShape: Finished. Vertices: " << out.vertices.size() / 3
            << ", Indices: " << out.indices.size();

        return out;
    }

} // namespace Urbaxio::CadKernel 