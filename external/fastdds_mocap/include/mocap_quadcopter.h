// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*!
 * @file mocap_quadcopter.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _MOCAP_QUADCOPTER_H_
#define _MOCAP_QUADCOPTER_H_


#include <stdint.h>
#include <array>
#include <string>
#include <vector>
#include <map>
#include <bitset>

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#define eProsima_user_DllExport __declspec( dllexport )
#else
#define eProsima_user_DllExport
#endif  // EPROSIMA_USER_DLL_EXPORT
#else
#define eProsima_user_DllExport
#endif  // _WIN32

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#if defined(mocap_quadcopter_SOURCE)
#define mocap_quadcopter_DllAPI __declspec( dllexport )
#else
#define mocap_quadcopter_DllAPI __declspec( dllimport )
#endif // mocap_quadcopter_SOURCE
#else
#define mocap_quadcopter_DllAPI
#endif  // EPROSIMA_USER_DLL_EXPORT
#else
#define mocap_quadcopter_DllAPI
#endif // _WIN32

namespace eprosima {
namespace fastcdr {
class Cdr;
} // namespace fastcdr
} // namespace eprosima


/*!
 * @brief This class represents the structure mocap_quadcopter defined by the user in the IDL file.
 * @ingroup MOCAP_QUADCOPTER
 */
class mocap_quadcopter
{
public:

    /*!
     * @brief Default constructor.
     */
    eProsima_user_DllExport mocap_quadcopter();

    /*!
     * @brief Default destructor.
     */
    eProsima_user_DllExport ~mocap_quadcopter();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object mocap_quadcopter that will be copied.
     */
    eProsima_user_DllExport mocap_quadcopter(
            const mocap_quadcopter& x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object mocap_quadcopter that will be copied.
     */
    eProsima_user_DllExport mocap_quadcopter(
            mocap_quadcopter&& x);

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object mocap_quadcopter that will be copied.
     */
    eProsima_user_DllExport mocap_quadcopter& operator =(
            const mocap_quadcopter& x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object mocap_quadcopter that will be copied.
     */
    eProsima_user_DllExport mocap_quadcopter& operator =(
            mocap_quadcopter&& x);

    /*!
     * @brief This function sets a value in member index
     * @param _index New value for member index
     */
    eProsima_user_DllExport void index(
            uint32_t _index);

    /*!
     * @brief This function returns the value of member index
     * @return Value of member index
     */
    eProsima_user_DllExport uint32_t index() const;

    /*!
     * @brief This function returns a reference to member index
     * @return Reference to member index
     */
    eProsima_user_DllExport uint32_t& index();

    /*!
     * @brief This function copies the value in member position
     * @param _position New value to be copied in member position
     */
    eProsima_user_DllExport void position(
            const std::array<double, 3>& _position);

    /*!
     * @brief This function moves the value in member position
     * @param _position New value to be moved in member position
     */
    eProsima_user_DllExport void position(
            std::array<double, 3>&& _position);

    /*!
     * @brief This function returns a constant reference to member position
     * @return Constant reference to member position
     */
    eProsima_user_DllExport const std::array<double, 3>& position() const;

    /*!
     * @brief This function returns a reference to member position
     * @return Reference to member position
     */
    eProsima_user_DllExport std::array<double, 3>& position();
    /*!
     * @brief This function copies the value in member orientation_quaternion
     * @param _orientation_quaternion New value to be copied in member orientation_quaternion
     */
    eProsima_user_DllExport void orientation_quaternion(
            const std::array<double, 4>& _orientation_quaternion);

    /*!
     * @brief This function moves the value in member orientation_quaternion
     * @param _orientation_quaternion New value to be moved in member orientation_quaternion
     */
    eProsima_user_DllExport void orientation_quaternion(
            std::array<double, 4>&& _orientation_quaternion);

    /*!
     * @brief This function returns a constant reference to member orientation_quaternion
     * @return Constant reference to member orientation_quaternion
     */
    eProsima_user_DllExport const std::array<double, 4>& orientation_quaternion() const;

    /*!
     * @brief This function returns a reference to member orientation_quaternion
     * @return Reference to member orientation_quaternion
     */
    eProsima_user_DllExport std::array<double, 4>& orientation_quaternion();
    /*!
     * @brief This function copies the value in member orientation_euler
     * @param _orientation_euler New value to be copied in member orientation_euler
     */
    eProsima_user_DllExport void orientation_euler(
            const std::array<double, 3>& _orientation_euler);

    /*!
     * @brief This function moves the value in member orientation_euler
     * @param _orientation_euler New value to be moved in member orientation_euler
     */
    eProsima_user_DllExport void orientation_euler(
            std::array<double, 3>&& _orientation_euler);

    /*!
     * @brief This function returns a constant reference to member orientation_euler
     * @return Constant reference to member orientation_euler
     */
    eProsima_user_DllExport const std::array<double, 3>& orientation_euler() const;

    /*!
     * @brief This function returns a reference to member orientation_euler
     * @return Reference to member orientation_euler
     */
    eProsima_user_DllExport std::array<double, 3>& orientation_euler();
    /*!
     * @brief This function sets a value in member delay
     * @param _delay New value for member delay
     */
    eProsima_user_DllExport void delay(
            float _delay);

    /*!
     * @brief This function returns the value of member delay
     * @return Value of member delay
     */
    eProsima_user_DllExport float delay() const;

    /*!
     * @brief This function returns a reference to member delay
     * @return Reference to member delay
     */
    eProsima_user_DllExport float& delay();

    /*!
     * @brief This function copies the value in member object_name
     * @param _object_name New value to be copied in member object_name
     */
    eProsima_user_DllExport void object_name(
            const std::string& _object_name);

    /*!
     * @brief This function moves the value in member object_name
     * @param _object_name New value to be moved in member object_name
     */
    eProsima_user_DllExport void object_name(
            std::string&& _object_name);

    /*!
     * @brief This function returns a constant reference to member object_name
     * @return Constant reference to member object_name
     */
    eProsima_user_DllExport const std::string& object_name() const;

    /*!
     * @brief This function returns a reference to member object_name
     * @return Reference to member object_name
     */
    eProsima_user_DllExport std::string& object_name();

    /*!
     * @brief This function returns the maximum serialized size of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getMaxCdrSerializedSize(
            size_t current_alignment = 0);

    /*!
     * @brief This function returns the serialized size of a data depending on the buffer alignment.
     * @param data Data which is calculated its serialized size.
     * @param current_alignment Buffer alignment.
     * @return Serialized size.
     */
    eProsima_user_DllExport static size_t getCdrSerializedSize(
            const mocap_quadcopter& data,
            size_t current_alignment = 0);


    /*!
     * @brief This function serializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serialize(
            eprosima::fastcdr::Cdr& cdr) const;

    /*!
     * @brief This function deserializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void deserialize(
            eprosima::fastcdr::Cdr& cdr);



    /*!
     * @brief This function returns the maximum serialized size of the Key of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getKeyMaxCdrSerializedSize(
            size_t current_alignment = 0);

    /*!
     * @brief This function tells you if the Key has been defined for this type
     */
    eProsima_user_DllExport static bool isKeyDefined();

    /*!
     * @brief This function serializes the key members of an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serializeKey(
            eprosima::fastcdr::Cdr& cdr) const;

private:

    uint32_t m_index;
    std::array<double, 3> m_position;
    std::array<double, 4> m_orientation_quaternion;
    std::array<double, 3> m_orientation_euler;
    float m_delay;
    std::string m_object_name;
};

#endif // _MOCAP_QUADCOPTER_H_