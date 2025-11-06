//
//  ArrayExtension.swift
//  RayTracer
//
//  Created by Eren Demircan on 2.11.2025.
//

extension Array {
    subscript(_ index: UInt) -> Element {
        get { self[Int(index)] }
        set { self[Int(index)] = newValue }
    }
}
