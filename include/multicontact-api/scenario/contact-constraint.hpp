// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_scenario_contact_constraint_hpp__
#define __multicontact_api_scenario_contact_constraint_hpp__

#include "multicontact-api/scenario/constraint.hpp"

namespace multicontact_api
{
  namespace scenario
  {

    template<typename Derived>
    struct ContactConstraintBase : ConstraintBase<Derived>
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      typedef typename traits<Derived>::ContactModel ContactModel;

      ContactConstraintBase()
      : m_contact_model(ContactModel())
      {}

      ContactConstraintBase(const ContactModel & contact_model)
      : m_contact_model(contact_model)
      {}

      void setContactModel(const ContactModel & contact_model)
      { m_contact_model = contact_model; }

      const ContactModel & contactModel() const { return m_contact_model; }
      ContactModel & contactModel() { return m_contact_model; }

    protected:

      ContactModel m_contact_model;

    };
  }
}

#endif // ifndef __multicontact_api_scenario_contact_constraint_hpp__
