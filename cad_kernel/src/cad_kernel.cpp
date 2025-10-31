#include "cad_kernel/cad_kernel.h"
#include <Standard_Macro.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Version.hxx>
#include <Standard_Failure.hxx>
#include <Message.hxx>
#include <Message_PrinterOStream.hxx>
#include <Message_Messenger.hxx>
#include <gp_Pnt.hxx>
#include <gp_Ax2.hxx>
#include <TopoDS_Shape.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <exception>
#include <iostream>

namespace Urbaxio::CadKernel {

    // ---                                   ---
    //                                   .h
    void ShapeDeleter::operator()(TopoDS_Shape* shape) const {
        if (shape) {
            std::cout << "CAD Kernel: Deleting shape via ShapeDeleter (raw pointer: " << shape << ")" << std::endl;
            delete shape;
        }
    }

    // ---               (             ) ---
    void initialize() {
        Message::SendInfo() << "CAD Kernel: Initializing OpenCascade Technology (OCCT) v"
            << OCC_VERSION_STRING_EXT << "...";
        try {
            Message::SendInfo() << "CAD Kernel: Standard Allocator initialization skipped (not needed).";
            Handle(Message_Messenger) aMessenger = Message::DefaultMessenger();
            if (aMessenger->Printers().IsEmpty()) {
                Handle(Message_Printer) aPrinter = new Message_PrinterOStream;
                aMessenger->AddPrinter(aPrinter);
                Message::SendInfo() << "CAD Kernel: Default Messenger and OStream Printer Initialized.";
            }
            else {
                Message::SendInfo() << "CAD Kernel: Default Messenger already has printers.";
            }
        }
        catch (Standard_Failure& e) {
            Message::SendFail() << "CAD Kernel: OCCT Standard_Failure during extra initialization: " << e.GetMessageString();
        }
        catch (...) {
            Message::SendFail() << "CAD Kernel: Unknown exception during extra initialization.";
        }
    }

    // --- create_box (             ) ---
    OCCT_ShapeUniquePtr create_box(double dx, double dy, double dz) {
        Message::SendInfo() << "CAD Kernel: Attempting to create box with dimensions: "
            << dx << ", " << dy << ", " << dz;
        Standard_ErrorHandler aErrorHandler;
        try {
            OCC_CATCH_SIGNALS
                if (dx <= 0 || dy <= 0 || dz <= 0) { return nullptr; }
            // --- MODIFIED LINE: Create a centered box ---
            BRepPrimAPI_MakeBox mkBox(gp_Pnt(-dx / 2.0, -dy / 2.0, -dz / 2.0), gp_Pnt(dx / 2.0, dy / 2.0, dz / 2.0));
            mkBox.Build();
            if (!mkBox.IsDone()) {
                Message::SendFail() << "CAD Kernel: Error! BRepPrimAPI_MakeBox failed. IsDone() returned false.";
                return nullptr;
            }
            if (mkBox.Shape().IsNull()) {
                Message::SendFail() << "CAD Kernel: Error! BRepPrimAPI_MakeBox succeeded but produced a Null shape.";
                return nullptr;
            }
            TopoDS_Shape* shape_copy = new TopoDS_Shape(mkBox.Shape());
            Message::SendInfo() << "CAD Kernel: Box created successfully (raw pointer: " << shape_copy << ")";
            return OCCT_ShapeUniquePtr(shape_copy);
        }
        catch (Standard_Failure& e) {
            Message::SendFail() << "CAD Kernel: Caught OCCT Standard_Failure exception during box creation.";
            Message::SendFail() << "CAD Kernel: OCCT Exception Message: " << e.GetMessageString();
            return nullptr;
        }
        catch (...) {
            Message::SendFail() << "CAD Kernel: Caught unknown exception during box creation.";
            return nullptr;
        }
    }

} // namespace Urbaxio::CadKernel 